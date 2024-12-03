import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import logging
import csv

# Configure Logging
logging.basicConfig(level=logging.INFO,
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
SAWTOOTH_PERIOD = 2  # Period in seconds

# GPIO Pins for Motor 2 (BCM numbering)
MOTOR2_IN1 = 25    # was 29
MOTOR2_IN2 = 5   # was 22
MOTOR2_SPD = 6    # was 31
MOTOR2_ADC_CHANNEL = 1

class SpikeFilter:
    def __init__(self, name, enabled=True):
        self.filter_active = False
        self.in_dead_zone = False
        self.last_valid_reading = None
        self.name = name
        self.enabled = enabled
        # Constants based on requirements
        self.DEAD_ZONE_THRESHOLD = 950  # Activation threshold before dead zone
        self.LOWER_VALID_LIMIT = 150    # ~45 degrees
        self.UPPER_VALID_LIMIT = 750    # Balanced threshold for backward movement
        self.ADC_MAX = 1023             # Maximum ADC reading

    def filter(self, new_value):
        if not self.enabled:
            # Spike filter is disabled, return the raw value
            self.last_valid_reading = new_value
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
    def __init__(self, name, in1, in2, pwm, adc_channel, mcp, spike_filter_enabled=True):
        self.name = name
        self.in1 = in1
        self.in2 = in2
        self.pwm = pwm
        self.adc_channel = adc_channel
        self.mcp = mcp
        self.position = 0
        self.last_valid_position = None
        self.pid = PIDController(Kp=0.8, Ki=0.1, Kd=0.05, name=f'PID-{self.name}')
        self.spike_filter = SpikeFilter(name, enabled=spike_filter_enabled)
        self.previous_control_signal = 0.0  # For rate limiting
        self.encoder_flipped = True  # Adjust if necessary

    def read_position(self):
        raw_value = self.mcp.read_adc(self.adc_channel)

        if self.encoder_flipped:
            raw_value = ADC_MAX - raw_value

        filtered_value = self.spike_filter.filter(raw_value)

        if filtered_value is None:
            # In dead zone, assume position is 0 degrees
            logging.debug(f"{self.name} is in dead zone. Position assumed to be 0°.")
            return 0.0
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

    def move_to_position(self, target):
        try:
            current_position = self.read_position()
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
                return True, error, control_signal, current_position

            direction = 'forward' if control_signal > 0 else 'backward'
            self.set_motor_direction(direction)
            speed = min(max_speed, max(30, abs(control_signal)))  # Apply clamping
            self.pwm.ChangeDutyCycle(speed)
            logging.debug(f"{self.name} Speed: {speed}% (Control Signal: {control_signal})")
            return False, error, control_signal, current_position
        except Exception as e:
            logging.error(f"{self.name} MotorController: Error during move_to_position: {e}")
            self.reset()
            return False, 0, 0, 0

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

def main(spike_filter_enabled, csv_filename):
    # Initialize GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MOTOR2_IN1, GPIO.OUT)
    GPIO.setup(MOTOR2_IN2, GPIO.OUT)
    GPIO.setup(MOTOR2_SPD, GPIO.OUT)

    # Set up PWM for motor speed control
    motor2_pwm = GPIO.PWM(MOTOR2_SPD, 1000)
    motor2_pwm.start(0)

    # Set up MCP3008
    mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

    # Initialize motor controller
    motor2 = MotorController("M2", MOTOR2_IN1, MOTOR2_IN2, motor2_pwm,
                             MOTOR2_ADC_CHANNEL, mcp, spike_filter_enabled=spike_filter_enabled)

    start_time = time.time()
    duration = 10  # Run the test for 10 seconds
    data_records = []

    try:
        while (time.time() - start_time) < duration:
            # Generate target position (sawtooth wave)
            elapsed_time = time.time() - start_time
            position_in_cycle = (elapsed_time % SAWTOOTH_PERIOD) / SAWTOOTH_PERIOD
            target_position = (position_in_cycle * MAX_ANGLE) % MAX_ANGLE

            # Move motor to target position in negative direction (backward)
            finished, error, control_signal, actual_position = motor2.move_to_position(target_position)

            # Collect data
            data_records.append({
                'Time': elapsed_time,
                'Error (degrees)': error,
                'Control Signal (%)': control_signal,
                'Target Position (degrees)': target_position,
                'Actual Position (degrees)': actual_position
            })

            time.sleep(0.02)  # 20 ms delay
    finally:
        # Stop the motor and cleanup
        motor2.stop_motor()
        motor2_pwm.stop()
        GPIO.cleanup()

        # Write data to CSV file
        with open(csv_filename, 'w', newline='') as csvfile:
            fieldnames = ['Time', 'Error (degrees)', 'Control Signal (%)', 'Target Position (degrees)', 'Actual Position (degrees)']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            writer.writeheader()
            for record in data_records:
                writer.writerow(record)

        logging.info(f"Data written to {csv_filename}")

if __name__ == "__main__":
    # Run with spike filter enabled
    logging.info("Starting test with spike filter enabled.")
    main(spike_filter_enabled=True, csv_filename='with_spike_filter.csv')

    # Run with spike filter disabled
    logging.info("Starting test with spike filter disabled.")
    main(spike_filter_enabled=False, csv_filename='without_spike_filter.csv')
