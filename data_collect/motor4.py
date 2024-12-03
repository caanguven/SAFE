import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import csv
import math
import logging

# Configure Logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s [%(levelname)s] %(message)s',
                    handlers=[
                        logging.FileHandler("motor4_test.log"),
                        logging.StreamHandler()
                    ])

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

# Time settings
TEST_DURATION = 10  # seconds
CONTROL_LOOP_DELAY = 0.02  # 20 ms

class SpikeFilter:
    def __init__(self, name):
        self.filter_active = False
        self.in_dead_zone = False
        self.last_valid_reading = None
        self.name = name
        # Constants based on requirements
        self.DEAD_ZONE_THRESHOLD = 950  # Activation threshold before dead zone
        self.LOWER_VALID_LIMIT = 150    # ~45 degrees
        self.UPPER_VALID_LIMIT = 750    # Balanced threshold for backward movement
        self.ADC_MAX = 1023             # Maximum ADC reading

    def filter(self, new_value):
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
    def __init__(self, name, in1, in2, pwm, adc_channel, mcp, encoder_flipped=False, spike_filter_active=True):
        self.name = name
        self.in1 = in1
        self.in2 = in2
        self.pwm = pwm
        self.adc_channel = adc_channel
        self.mcp = mcp
        self.position = 0
        self.last_valid_position = None
        self.pid = PIDController(Kp=0.8, Ki=0.1, Kd=0.05, name=f'PID-{self.name}')
        self.encoder_flipped = encoder_flipped
        self.spike_filter = SpikeFilter(name) if spike_filter_active else None
        self.spike_filter_active = spike_filter_active
        self.previous_control_signal = 0.0  # For rate limiting

    def read_position(self):
        raw_value = self.mcp.read_adc(self.adc_channel)
        
        if self.encoder_flipped:
            raw_value = ADC_MAX - raw_value
        
        if self.spike_filter and self.spike_filter_active:
            filtered_value = self.spike_filter.filter(raw_value)
        else:
            filtered_value = raw_value  # No filtering

        if filtered_value is None:
            # In dead zone, assume position is 0 degrees
            logging.debug(f"{self.name} is in dead zone. Position assumed to be 0°.")
            degrees = 0.0
        else:
            degrees = (filtered_value / ADC_MAX) * MAX_ANGLE
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

    def move_backward(self, target, control_data, start_time):
        try:
            current_position = self.read_position()
            error = target - current_position

            # Adjust error for wrap-around
            if error > MAX_ANGLE / 2:
                error -= MAX_ANGLE
            elif error < -MAX_ANGLE / 2:
                error += MAX_ANGLE

            control_signal = self.pid.compute(error)

            # Rate limiting and signal clamping during dead zone
            if self.spike_filter and self.spike_filter_active and self.spike_filter.in_dead_zone:
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
                logging.info(f"{self.name} reached target. Stopping motor.")
                return True  # Target reached

            direction = 'forward' if control_signal > 0 else 'backward'
            self.set_motor_direction(direction)
            speed = min(max_speed, max(30, abs(control_signal)))  # Apply clamping
            self.pwm.ChangeDutyCycle(speed)
            logging.debug(f"{self.name} Speed: {speed}% (Control Signal: {control_signal})")

            # Record data
            elapsed_time = time.time() - start_time
            target_path = generate_sawtooth_position(elapsed_time)
            control_data.append({
                'error_degrees': error,
                'control_signal_percent': speed,
                'target_path_degrees': target_path,
                'actual_position_degrees': current_position
            })

            return False  # Continue moving

        except Exception as e:
            logging.error(f"{self.name} MotorController: Error during move_backward: {e}")
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
        if self.spike_filter:
            self.spike_filter.reset()
        self.previous_control_signal = 0.0  # Reset previous control signal
        logging.info(f"{self.name} MotorController: Reset complete.")

def generate_sawtooth_position(elapsed_time):
    position_in_cycle = (elapsed_time % SAWTOOTH_PERIOD) / SAWTOOTH_PERIOD
    position = (position_in_cycle * MAX_ANGLE) % MAX_ANGLE
    return position

def run_motor_test(spike_filter_active, csv_filename):
    logging.info(f"Starting motor test with spike_filter_active={spike_filter_active}")
    
    # GPIO setup
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MOTOR4_IN1, GPIO.OUT)
    GPIO.setup(MOTOR4_IN2, GPIO.OUT)
    GPIO.setup(MOTOR4_SPD, GPIO.OUT)

    # Set up PWM for motor speed control
    motor4_pwm = GPIO.PWM(MOTOR4_SPD, 1000)  # 1 kHz
    motor4_pwm.start(0)

    # Set up MCP3008
    mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

    # Initialize Motor4 Controller
    motor4 = MotorController(
        name="M4",
        in1=MOTOR4_IN1,
        in2=MOTOR4_IN2,
        pwm=motor4_pwm,
        adc_channel=MOTOR4_ADC_CHANNEL,
        mcp=mcp,
        encoder_flipped=True,
        spike_filter_active=spike_filter_active
    )

    # Prepare CSV file
    with open(csv_filename, mode='w', newline='') as csv_file:
        fieldnames = ['time_sec', 'error_degrees', 'control_signal_percent', 'target_path_degrees', 'actual_position_degrees']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        control_data = []
        start_time = time.time()
        end_time = start_time + TEST_DURATION
        target_reached = False

        # Set motor direction to backward
        motor4.set_motor_direction('backward')

        while time.time() < end_time and not target_reached:
            elapsed_time = time.time() - start_time
            target_position = generate_sawtooth_position(elapsed_time)
            target_reached = motor4.move_backward(target_position, control_data, start_time)
            time.sleep(CONTROL_LOOP_DELAY)

        # Stop motor after test
        motor4.stop_motor()

        # Write collected data to CSV
        for entry in control_data:
            writer.writerow({
                'time_sec': round(elapsed_time, 2),
                'error_degrees': round(entry['error_degrees'], 2),
                'control_signal_percent': round(entry['control_signal_percent'], 2),
                'target_path_degrees': round(entry['target_path_degrees'], 2),
                'actual_position_degrees': round(entry['actual_position_degrees'], 2)
            })

    # Cleanup
    motor4.reset()
    motor4_pwm.stop()
    GPIO.cleanup()
    logging.info(f"Motor test completed. Data saved to {csv_filename}")

def main():
    try:
        # Run test with Spike Filter Activated
        run_motor_test(
            spike_filter_active=True,
            csv_filename='motor4_with_spike_filter.csv'
        )

        # Small delay between tests
        time.sleep(2)

        # Run test without Spike Filter
        run_motor_test(
            spike_filter_active=False,
            csv_filename='motor4_without_spike_filter.csv'
        )

    except KeyboardInterrupt:
        logging.info("Test interrupted by user.")
    except Exception as e:
        logging.error(f"An error occurred: {e}")
    finally:
        GPIO.cleanup()
        logging.info("GPIO cleanup completed.")

if __name__ == "__main__":
    main()
