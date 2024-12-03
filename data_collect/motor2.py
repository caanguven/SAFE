import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import threading
import logging
import csv

# Configure Logging
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s [%(levelname)s] %(message)s',
                    handlers=[
                        logging.FileHandler("motor_control.log"),
                        logging.StreamHandler()
                    ])

# Constants for SPI and ADC
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023
MIN_ANGLE = 0
MAX_ANGLE = 330
SAWTOOTH_PERIOD = 3  # Period in seconds

# GPIO Pins for Motor 2 (BCM numbering)
MOTOR2_IN1 = 5    # was 29
MOTOR2_IN2 = 25   # was 22
MOTOR2_SPD = 6    # was 31
MOTOR2_ADC_CHANNEL = 1

class SpikeFilter:
    def __init__(self, name):
        self.filter_active = False
        self.last_valid_reading = None
        self.name = name
        # Adding constants based on requirements
        self.DEAD_ZONE_THRESHOLD = 950  # Activation threshold before dead zone
        self.LOWER_VALID_LIMIT = 150    # ~45 degrees
        self.UPPER_VALID_LIMIT = 750    # Balanced threshold for backward movement
        self.ADC_MAX = 1023             # Maximum ADC reading

    def filter(self, new_value):
        # Case 1: Check for entering dead zone
        if not self.filter_active and self.last_valid_reading is not None:
            if self.last_valid_reading >= self.DEAD_ZONE_THRESHOLD:
                self.filter_active = True
                logging.debug(f"{self.name} SpikeFilter: Entering dead zone. Last valid: {self.last_valid_reading}")
                return self.last_valid_reading

        # Case 2: Filter is active (in dead zone)
        if self.filter_active:
            # Check if we've exited the dead zone with a valid reading
            if self.LOWER_VALID_LIMIT <= new_value <= self.UPPER_VALID_LIMIT:
                self.filter_active = False
                self.last_valid_reading = new_value
                logging.debug(f"{self.name} SpikeFilter: Exited dead zone with valid reading {new_value}")
                return new_value
            else:
                # Still in dead zone, return last valid reading
                return self.last_valid_reading if self.last_valid_reading is not None else 0

        # Case 3: Normal operation (filter not active)
        # Check for sudden spikes that might indicate entering dead zone
        if self.last_valid_reading is not None and abs(self.last_valid_reading - new_value) > 300:
            self.filter_active = True
            logging.debug(f"{self.name} SpikeFilter: Sudden spike detected: {new_value}")
            return self.last_valid_reading

        # Normal valid reading
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

        logging.debug(f"{self.name} PID: error={error}, proportional={proportional}, integral={integral}, derivative={derivative}, control_signal={control_signal}")

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
        self.spike_filter_enabled = True  # Initially enabled

    def enable_spike_filter(self):
        self.spike_filter_enabled = True
        self.spike_filter.reset()
        logging.info(f"{self.name} SpikeFilter: Enabled.")

    def disable_spike_filter(self):
        self.spike_filter_enabled = False
        self.spike_filter.reset()
        logging.info(f"{self.name} SpikeFilter: Disabled.")

    def read_position(self, mcp):
        raw_value = mcp.read_adc(self.adc_channel)
        
        if self.encoder_flipped:
            raw_value = ADC_MAX - raw_value
        
        if self.spike_filter_enabled:
            filtered_value = self.spike_filter.filter(raw_value)
            if filtered_value is None:
                return self.last_valid_position if self.last_valid_position is not None else 0
            degrees = (filtered_value / ADC_MAX) * 330.0
            self.last_valid_position = degrees
            logging.debug(f"{self.name} Position (Filtered): {degrees}° (raw: {raw_value})")
            return degrees
        else:
            degrees = (raw_value / ADC_MAX) * 330.0
            logging.debug(f"{self.name} Position (Raw): {degrees}° (raw: {raw_value})")
            return degrees

    def set_motor_direction(self, direction):
        if direction == 'forward':
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
            logging.debug(f"{self.name} Direction: Forward")
        elif direction == 'backward':
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
            logging.debug(f"{self.name} Direction: Backward")
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.LOW)
            logging.debug(f"{self.name} Direction: Stopped")

    def move_to_position(self, target, mcp, direction_override=None):
        try:
            current_position = self.read_position(mcp)
            error = target - current_position

            # Handle wrap-around for angles
            if error > 165:
                error -= 330
            elif error < -165:
                error += 330

            control_signal = self.pid.compute(error)

            if abs(error) <= 2:
                self.stop_motor()
                logging.debug(f"{self.name} reached target. Stopping motor.")
                return True

            # Override direction if specified
            if direction_override:
                direction = direction_override
            else:
                direction = 'forward' if control_signal > 0 else 'backward'

            self.set_motor_direction(direction)
            speed = min(100, max(30, abs(control_signal)))
            self.pwm.ChangeDutyCycle(speed)
            logging.debug(f"{self.name} Speed: {speed}%")
            return False
        except Exception as e:
            logging.error(f"{self.name} MotorController: Error during move_to_position: {e}")
            self.reset()
            return False

    def stop_motor(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
        logging.debug(f"{self.name} Motor Stopped")

    def reset(self):
        self.stop_motor()
        self.pid.reset()
        self.spike_filter.reset()
        logging.info(f"{self.name} MotorController: Reset complete.")

class MotorControlSystem:
    def __init__(self):
        self.running = True
        self.lock = threading.Lock()

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(MOTOR2_IN1, GPIO.OUT)
        GPIO.setup(MOTOR2_IN2, GPIO.OUT)
        GPIO.setup(MOTOR2_SPD, GPIO.OUT)

        # Set up PWM for motor speed control
        self.motor2_pwm = GPIO.PWM(MOTOR2_SPD, 1000)
        self.motor2_pwm.start(0)

        # Set up MCP3008
        self.mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

        # Initialize Motor 2
        self.motor2 = MotorController("M2", MOTOR2_IN1, MOTOR2_IN2, self.motor2_pwm,
                                      MOTOR2_ADC_CHANNEL, encoder_flipped=True)

        self.start_time = time.time()

        # Initialize CSV files
        self.csv_with_filter = open('with_spike_filter.csv', mode='w', newline='')
        self.csv_without_filter = open('without_spike_filter.csv', mode='w', newline='')
        self.writer_with_filter = csv.writer(self.csv_with_filter)
        self.writer_without_filter = csv.writer(self.csv_without_filter)

        # Write headers
        headers = ['Error_to_Target_deg', 'Control_Signal_%', 'Target_Path_Sawtooth_deg', 'Actual_Position_deg']
        self.writer_with_filter.writerow(headers)
        self.writer_without_filter.writerow(headers)

        # Start the control loop in a separate thread
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()

        logging.info("MotorControlSystem initialized and control loop started.")

    def generate_sawtooth_position(self):
        elapsed_time = time.time() - self.start_time
        position_in_cycle = (elapsed_time % SAWTOOTH_PERIOD) / SAWTOOTH_PERIOD
        position = (position_in_cycle * MAX_ANGLE) % MAX_ANGLE
        return position

    def control_loop(self):
        try:
            while self.running:
                elapsed_time = time.time() - self.start_time

                # Phase 1: First 15 seconds with spike filter active
                if elapsed_time < 15:
                    if not self.motor2.spike_filter_enabled:
                        self.motor2.enable_spike_filter()
                    target_position = self.generate_sawtooth_position()

                    # Read filtered position
                    actual_position = self.motor2.read_position(self.mcp)
                    error = target_position - actual_position

                    # Handle wrap-around for angles
                    if error > 165:
                        error -= 330
                    elif error < -165:
                        error += 330

                    control_signal = self.motor2.pid.compute(error)

                    # Set direction to 'backward' to control motor in negative direction
                    direction = 'backward'
                    self.motor2.set_motor_direction(direction)
                    speed = min(100, max(30, abs(control_signal)))
                    self.motor2.pwm.ChangeDutyCycle(speed)
                    logging.debug(f"{self.motor2.name} Speed: {speed}% (Control Signal: {control_signal})")

                    # Log to CSV with spike filter
                    self.writer_with_filter.writerow([
                        round(error, 2),
                        round(control_signal, 2),
                        round(target_position, 2),
                        round(actual_position, 2)
                    ])
                    self.csv_with_filter.flush()

                # Phase 2: Next 15 seconds without spike filter
                elif 15 <= elapsed_time < 30:
                    if self.motor2.spike_filter_enabled:
                        self.motor2.disable_spike_filter()
                    target_position = self.generate_sawtooth_position()

                    # Read raw position
                    actual_position = self.motor2.read_position(self.mcp)
                    error = target_position - actual_position

                    # Handle wrap-around for angles
                    if error > 165:
                        error -= 330
                    elif error < -165:
                        error += 330

                    control_signal = self.motor2.pid.compute(error)

                    # Set direction to 'backward' to control motor in negative direction
                    direction = 'backward'
                    self.motor2.set_motor_direction(direction)
                    speed = min(100, max(30, abs(control_signal)))
                    self.motor2.pwm.ChangeDutyCycle(speed)
                    logging.debug(f"{self.motor2.name} Speed: {speed}% (Control Signal: {control_signal})")

                    # Log to CSV without spike filter
                    self.writer_without_filter.writerow([
                        round(error, 2),
                        round(control_signal, 2),
                        round(target_position, 2),
                        round(actual_position, 2)
                    ])
                    self.csv_without_filter.flush()

                # Terminate after 30 seconds
                elif elapsed_time >= 30:
                    logging.info("Completed 30 seconds of motor control. Stopping control loop.")
                    self.stop()
                    break

                time.sleep(0.02)  # 20 ms delay

        except Exception as e:
            logging.error(f"Control Loop Error: {e}")
            self.stop()

    def stop(self):
        with self.lock:
            if not self.running:
                return  # Prevent multiple calls
            self.running = False
        self.control_thread.join()
        self.motor2.stop_motor()
        self.motor2_pwm.stop()
        GPIO.cleanup()
        self.csv_with_filter.close()
        self.csv_without_filter.close()
        logging.info("MotorControlSystem stopped and resources cleaned up.")

if __name__ == "__main__":
    try:
        motor_control_system = MotorControlSystem()
        logging.info("Motor Control System is running. It will stop automatically after 30 seconds.")
        while motor_control_system.running:
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt received. Stopping Motor Control System.")
        motor_control_system.stop()
    except Exception as e:
        logging.error(f"Unexpected error: {e}")
        motor_control_system.stop()
