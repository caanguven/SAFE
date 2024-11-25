# motor_control.py

import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import threading
import logging

# Constants for SPI and ADC
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023
MIN_ANGLE = 0
MAX_ANGLE = 330
SAWTOOTH_PERIOD = 2  # Period in seconds

# GPIO Pins for Motor 3
MOTOR3_IN1 = 11
MOTOR3_IN2 = 32
MOTOR3_SPD = 33
MOTOR3_ADC_CHANNEL = 2

class SpikeFilter:
    def __init__(self, name):
        self.filter_active = False
        self.last_valid_reading = None
        self.name = name

    def filter(self, new_value):
        if self.filter_active:
            if 150 <= new_value <= 700:
                logging.debug(f"{self.name} SpikeFilter: Value {new_value} within spike filter range, filtering out.")
                return None
            else:
                self.filter_active = False
                self.last_valid_reading = new_value
                logging.debug(f"{self.name} SpikeFilter: Spike deactivated with value {new_value}")
                return new_value
        else:
            if self.last_valid_reading is not None and abs(self.last_valid_reading - new_value) > 300:
                self.filter_active = True
                logging.debug(f"{self.name} SpikeFilter: Spike detected with value {new_value}")
                return None
            else:
                self.last_valid_reading = new_value
                return new_value

    def reset(self):
        self.filter_active = False
        self.last_valid_reading = None
        logging.debug(f"{self.name} SpikeFilter: Reset complete.")

class PIDController:
    def __init__(self, Kp, Ki, Kd, name='PID'):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()
        self.name = name

    def compute(self, error):
        current_time = time.time()
        delta_time = current_time - self.last_time
        if delta_time <= 0.0:
            delta_time = 0.0001

        proportional = self.Kp * error
        self.integral += error * delta_time
        integral = self.Ki * self.integral
        derivative = self.Kd * (error - self.previous_error) / delta_time

        control_signal = proportional + integral + derivative
        self.previous_error = error
        self.last_time = current_time

        logging.debug(f"{self.name} PID: error={error:.2f}, proportional={proportional:.2f}, "
                      f"integral={integral:.2f}, derivative={derivative:.2f}, control_signal={control_signal:.2f}%")

        return control_signal

    def reset(self):
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()
        logging.debug(f"{self.name} PID: Reset complete.")

class MotorController:
    def __init__(self, name, in1, in2, pwm, adc_channel, encoder_flipped=False):
        self.name = name
        self.in1 = in1
        self.in2 = in2
        self.pwm = pwm
        self.adc_channel = adc_channel
        self.position = 0
        self.last_valid_position = None
        self.pid = PIDController(Kp=0.8, Ki=0.1, Kd=0.05, name=f'PID-{self.name}')
        self.encoder_flipped = encoder_flipped
        self.spike_filter = SpikeFilter(name)
        self.use_spike_filter = True  # Flag to enable/disable spike filter

    def read_position(self, mcp):
        raw_value = mcp.read_adc(self.adc_channel)
        
        if self.encoder_flipped:
            raw_value = ADC_MAX - raw_value
        
        if self.use_spike_filter:
            filtered_value = self.spike_filter.filter(raw_value)
        else:
            filtered_value = raw_value  # No filtering

        if filtered_value is None:
            current_position = self.last_valid_position if self.last_valid_position is not None else 0
            logging.debug(f"{self.name} Position (Filtered): {current_position:.2f}° (raw: {raw_value})")
            return current_position
        else:
            degrees = (filtered_value / ADC_MAX) * MAX_ANGLE
            self.last_valid_position = degrees
            logging.debug(f"{self.name} Position: {degrees:.2f}° (raw: {raw_value})")
            return degrees

    def set_motor_direction(self, direction):
        if direction == 'forward':
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
            logging.debug(f"{self.name} Direction: Forward")
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
            logging.debug(f"{self.name} Direction: Backward")

    def move_to_position(self, target, mcp):
        try:
            current_position = self.read_position(mcp)
            error = target - current_position

            # Wrap error within [-165, 165] degrees for shortest path
            if error > 165:
                error -= 330
            elif error < -165:
                error += 330

            control_signal = self.pid.compute(error)

            if abs(error) <= 2:
                self.stop_motor()
                logging.info(f"{self.name} reached target. Stopping motor.")
                return True

            direction = 'forward' if control_signal > 0 else 'backward'
            self.set_motor_direction(direction)

            # Map control_signal to speed percentage between 30% and 100%
            speed = min(100, max(30, abs(control_signal)))
            self.pwm.ChangeDutyCycle(speed)
            logging.info(f"{self.name} | Raw: {mcp.read_adc(self.adc_channel)} | Angle: {current_position:.2f}° | "
                         f"Error: {error:.2f}° | Control Signal: {speed:.2f}%")
            return False
        except Exception as e:
            logging.error(f"{self.name} MotorController: Error during move_to_position: {e}")
            self.reset()
            return False

    def stop_motor(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
        logging.info(f"{self.name} Motor Stopped")

    def reset(self):
        self.stop_motor()
        self.pid.reset()
        self.spike_filter.reset()
        logging.info(f"{self.name} MotorController: Reset complete.")

class MotorControlSystem:
    def __init__(self, mode='normal', run_duration=30, spike_filter_duration=15):
        self.mode = mode  # 'normal' or 'gallop'
        self.current_direction = 'stable'
        self.lock = threading.Lock()
        self.running = True

        # Initialize separate loggers for with and without spike filter
        self.logger = logging.getLogger('Motor3Control')
        self.logger.setLevel(logging.DEBUG)
        self.logger.propagate = False  # Prevent double logging

        # Formatter
        formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s')

        # Handler for with spike filter
        self.handler_with_filter = logging.FileHandler('motor3_with_spike_filter.log')
        self.handler_with_filter.setLevel(logging.DEBUG)
        self.handler_with_filter.setFormatter(formatter)

        # Handler for without spike filter
        self.handler_without_filter = logging.FileHandler('motor3_without_spike_filter.log')
        self.handler_without_filter.setLevel(logging.DEBUG)
        self.handler_without_filter.setFormatter(formatter)

        # Initially, add only the with_spike_filter handler
        self.logger.addHandler(self.handler_with_filter)

        # Redirect logging to the custom logger
        logging.getLogger().handlers = []  # Remove other handlers
        logging.getLogger().addHandler(self.handler_with_filter)
        logging.getLogger().setLevel(logging.DEBUG)

        # GPIO setup
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(MOTOR3_IN1, GPIO.OUT)
        GPIO.setup(MOTOR3_IN2, GPIO.OUT)
        GPIO.setup(MOTOR3_SPD, GPIO.OUT)

        # Set up PWM for motor speed control
        self.motor3_pwm = GPIO.PWM(MOTOR3_SPD, 1000)
        self.motor3_pwm.start(0)

        # Set up MCP3008
        self.mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

        # Initialize Motor 3
        self.motor3 = MotorController("M3", MOTOR3_IN1, MOTOR3_IN2, self.motor3_pwm,
                                      MOTOR3_ADC_CHANNEL, encoder_flipped=False)

        self.motors = {
            'M3': self.motor3
        }

        self.start_time = time.time()
        self.run_duration = run_duration
        self.spike_filter_duration = spike_filter_duration  # Duration to keep spike filter active

        # Start the control loop in a separate thread
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()

        self.logger.info(f"MotorControlSystem initialized in '{self.mode}' mode.")

    def set_direction(self, direction):
        with self.lock:
            self.current_direction = direction
            self.logger.info(f"Direction set to '{direction}'.")
            if direction == 'stable':
                self.logger.info("Resetting Motor 3 to 'stable' state.")
                self.motors['M3'].reset()

    def set_mode(self, mode):
        with self.lock:
            self.mode = mode
            self.logger.info(f"Mode set to '{mode}'.")

    def get_status(self):
        with self.lock:
            status = {
                'mode': self.mode,
                'direction': self.current_direction,
                'motors': {}
            }

            for name, motor in self.motors.items():
                pos = motor.read_position(self.mcp)
                status['motors'][name] = pos

            return status

    def generate_sawtooth_position(self):
        elapsed_time = time.time() - self.start_time
        position_in_cycle = (elapsed_time % SAWTOOTH_PERIOD) / SAWTOOTH_PERIOD
        position = (position_in_cycle * MAX_ANGLE) % MAX_ANGLE
        return position

    def control_loop(self):
        spike_filter_disabled = False
        while self.running:
            current_time = time.time()
            elapsed_time = current_time - self.start_time

            # Disable spike filter after spike_filter_duration seconds
            if elapsed_time >= self.spike_filter_duration and not spike_filter_disabled:
                with self.lock:
                    for motor in self.motors.values():
                        motor.use_spike_filter = False
                        motor.spike_filter.reset()
                spike_filter_disabled = True

                # Switch logging handlers
                self.logger.removeHandler(self.handler_with_filter)
                self.logger.addHandler(self.handler_without_filter)
                logging.getLogger().removeHandler(self.handler_with_filter)
                logging.getLogger().addHandler(self.handler_without_filter)

                self.logger.info("Spike filters disabled. Running without spike filtering.")

            # Stop the system after run_duration seconds
            if elapsed_time >= self.run_duration:
                self.logger.info("Run duration completed. Preparing to stop MotorControlSystem.")
                self.running = False  # Signal to stop the loop
                break

            with self.lock:
                direction = self.current_direction
                mode = self.mode

            if direction != 'stable':
                # For single motor, we don't need motor groups
                motor = self.motors['M3']

                base_position = self.generate_sawtooth_position()

                # For a single motor, target is base_position
                target = base_position

                motor.move_to_position(target, self.mcp)
            else:
                # Stop Motor 3
                self.motors['M3'].stop_motor()
                self.logger.debug("Motor 3 stopped (direction: stable).")

            time.sleep(0.02)  # 20 ms delay

    def stop(self):
        if self.running:
            self.running = False
            self.control_thread.join()
            self.motors['M3'].stop_motor()

            self.motor3_pwm.stop()
            GPIO.cleanup()
            self.logger.info("MotorControlSystem stopped and GPIO cleaned up.")

def main():
    # Initialize MotorControlSystem
    motor_control_system = MotorControlSystem(mode='normal', run_duration=30, spike_filter_duration=15)

    # Set initial direction to 'forward' to start movement
    motor_control_system.set_direction('forward')

    try:
        # Wait until the control loop completes (30 seconds)
        while motor_control_system.control_thread.is_alive():
            time.sleep(1)
    except KeyboardInterrupt:
        logging.getLogger('Motor3Control').info("KeyboardInterrupt received. Stopping MotorControlSystem.")
        motor_control_system.stop()
    finally:
        if motor_control_system.control_thread.is_alive():
            motor_control_system.stop()

if __name__ == "__main__":
    main()
