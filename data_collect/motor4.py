import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import logging
import csv

# Configure Logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s [%(levelname)s] %(message)s')

# Constants for SPI and ADC
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023
MIN_ANGLE = 0
MAX_ANGLE = 330
SAWTOOTH_PERIOD = 2  # Period in seconds

# GPIO Pins for Motor 4 (BCM numbering)
MOTOR4_IN1 = 18
MOTOR4_IN2 = 27
MOTOR4_SPD = 19
MOTOR4_ADC_CHANNEL = 3

class SpikeFilter:
    def __init__(self, name, enabled=True):
        self.filter_active = False
        self.in_dead_zone = False
        self.last_valid_reading = None
        self.name = name
        self.enabled = enabled  # Added
        # Constants based on requirements
        self.DEAD_ZONE_THRESHOLD = 950  # Activation threshold before dead zone
        self.LOWER_VALID_LIMIT = 150    # ~45 degrees
        self.UPPER_VALID_LIMIT = 750    # Balanced threshold for backward movement
        self.ADC_MAX = 1023             # Maximum ADC reading

    def filter(self, new_value):
        if not self.enabled:
            return new_value

        # Case 1: Check for entering dead zone
        if not self.filter_active and self.last_valid_reading is not None:
            if self.last_valid_reading >= self.DEAD_ZONE_THRESHOLD:
                self.filter_active = True
                self.in_dead_zone = True
                logging.debug(f"{self.name} SpikeFilter: Entering dead zone. Last valid: {self.last_valid_reading}")
                return None  # Return None to indicate dead zone

        # Case 2: Filter is active (in dead zone)
        if self.filter_active:
            # Check if we've exited the dead zone with a valid reading
            if self.LOWER_VALID_LIMIT <= new_value <= self.UPPER_VALID_LIMIT:
                self.filter_active = False
                self.in_dead_zone = False
                self.last_valid_reading = new_value
                logging.debug(f"{self.name} SpikeFilter: Exited dead zone with valid reading {new_value}")
                return new_value
            else:
                # Still in dead zone, return None
                return None

        # Case 3: Normal operation (filter not active)
        # Check for sudden spikes that might indicate entering dead zone
        if self.last_valid_reading is not None and abs(self.last_valid_reading - new_value) > 300:
            self.filter_active = True
            self.in_dead_zone = True
            logging.debug(f"{self.name} SpikeFilter: Sudden spike detected: {new_value}")
            return None

        # Normal valid reading
        self.last_valid_reading = new_value
        return new_value

    def reset(self):
        self.filter_active = False
        self.in_dead_zone = False
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
        self.previous_control_signal = 0.0  # For rate limiting
        self.last_error = 0.0
        self.last_control_signal = 0.0

    def read_position(self, mcp):
        raw_value = mcp.read_adc(self.adc_channel)
        if self.encoder_flipped:
            raw_value = ADC_MAX - raw_value

        filtered_value = self.spike_filter.filter(raw_value)

        if filtered_value is None:
            # In dead zone, assume position is 0 degrees
            logging.debug(f"{self.name} is in dead zone. Position assumed to be 0°.")
            return 0.0
        else:
            degrees = (filtered_value / ADC_MAX) * 330.0
            self.last_valid_position = degrees
            logging.debug(f"{self.name} Position: {degrees}° (raw: {raw_value})")
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

            if error > 165:
                error -= 330
            elif error < -165:
                error += 330

            control_signal = self.pid.compute(error)

            # Rate limiting and signal clamping during dead zone
            if self.spike_filter.in_dead_zone:
                # Apply rate limiting
                max_change = 5  # Maximum allowed change in control_signal per cycle
                delta_signal = control_signal - self.previous_control_signal
                if delta_signal > max_change:
                    control_signal = self.previous_control_signal + max_change
                elif delta_signal < -max_change:
                    control_signal = self.previous_control_signal - max_change
                self.previous_control_signal = control_signal

                # Signal clamping to 50% of maximum speed
                max_speed = 50  # 50% of maximum speed
            else:
                max_speed = 100  # Full speed when not in dead zone
                self.previous_control_signal = control_signal  # Update previous control signal

            if abs(error) <= 2:
                self.stop_motor()
                logging.debug(f"{self.name} reached target. Stopping motor.")
                return True

            direction = 'forward' if control_signal > 0 else 'backward'
            self.set_motor_direction(direction)
            speed = min(max_speed, max(30, abs(control_signal)))  # Apply clamping
            self.pwm.ChangeDutyCycle(speed)
            logging.debug(f"{self.name} Speed: {speed}% (Control Signal: {control_signal})")

            # Store the last error and control signal
            self.last_error = error
            self.last_control_signal = control_signal

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
        self.previous_control_signal = 0.0  # Reset previous control signal
        logging.info(f"{self.name} MotorController: Reset complete.")

def generate_decreasing_sawtooth_position(start_time):
    elapsed_time = time.time() - start_time
    position_in_cycle = (elapsed_time % SAWTOOTH_PERIOD) / SAWTOOTH_PERIOD
    position = (MAX_ANGLE - (position_in_cycle * MAX_ANGLE)) % MAX_ANGLE
    return position

def main():
    try:
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(MOTOR4_IN1, GPIO.OUT)
        GPIO.setup(MOTOR4_IN2, GPIO.OUT)
        GPIO.setup(MOTOR4_SPD, GPIO.OUT)

        # Set up PWM for motor speed control
        motor4_pwm = GPIO.PWM(MOTOR4_SPD, 1000)
        motor4_pwm.start(0)

        # Set up MCP3008
        mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

        # Initialize motor 4
        motor4 = MotorController("M4", MOTOR4_IN1, MOTOR4_IN2, motor4_pwm,
                                 MOTOR4_ADC_CHANNEL, encoder_flipped=True)

        # Tests configurations
        tests = [
            {'spike_filter_enabled': True, 'csv_filename': 'motor4_forward_with_spike_filter.csv'},
            {'spike_filter_enabled': False, 'csv_filename': 'motor4_forward_without_spike_filter.csv'},
        ]

        for test in tests:
            motor4.reset()
            motor4.spike_filter.enabled = test['spike_filter_enabled']
            csv_filename = test['csv_filename']
            logging.info(f"Starting test with spike filter enabled: {test['spike_filter_enabled']}")

            with open(csv_filename, mode='w', newline='') as csvfile:
                fieldnames = ['Time (s)', 'Error (degrees)', 'Control Signal (%)', 'Target Position (degrees)', 'Actual Position (degrees)']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()

                start_time = time.time()
                end_time = start_time + 10  # Run for 10 seconds

                while time.time() < end_time:
                    current_time = time.time() - start_time
                    target_position = generate_decreasing_sawtooth_position(start_time)
                    motor4.move_to_position(target_position, mcp)
                    actual_position = motor4.read_position(mcp)
                    error = motor4.last_error
                    control_signal = motor4.last_control_signal

                    # Record data
                    writer.writerow({
                        'Time (s)': f"{current_time:.2f}",
                        'Error (degrees)': f"{error:.2f}",
                        'Control Signal (%)': f"{control_signal:.2f}",
                        'Target Position (degrees)': f"{target_position:.2f}",
                        'Actual Position (degrees)': f"{actual_position:.2f}"
                    })

                    time.sleep(0.02)  # 20 ms delay

            logging.info(f"Test completed and data saved to {csv_filename}")

    except KeyboardInterrupt:
        logging.info("Program interrupted by user.")
    except Exception as e:
        logging.error(f"An error occurred: {e}")
    finally:
        # Cleanup
        motor4.stop_motor()
        motor4_pwm.stop()
        GPIO.cleanup()
        logging.info("GPIO cleaned up.")

if __name__ == "__main__":
    main()
