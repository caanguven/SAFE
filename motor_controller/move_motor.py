import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import argparse
import logging

# ----------------------------
# Configuration and Constants
# ----------------------------

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),  # Log to console
        logging.FileHandler("motor_debug.log")  # Log to file
    ]
)

# Constants for SPI and ADC
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023
MAX_ANGLE = 330  # Maximum angle corresponding to ADC_MAX
DEAD_ZONE_THRESHOLD_NORMAL = 150  # For normal encoders (detect low ADC values)
DEAD_ZONE_THRESHOLD_INVERTED = 150  # For inverted encoders (detect high ADC values after inversion)
SAWTOOTH_PERIOD = 2  # Period in seconds
PHASE_SHIFT = 180  # Phase shift for Motor 3 and Motor 4 in degrees

# GPIO Pins for Motors
MOTORS_CONFIG = {
    'Motor1': {
        'in1': 7,
        'in2': 26,
        'spd': 18,
        'adc_channel': 0,
        'target': 180,
        'invert_encoder': False,
        'invert_direction': False,
        'phase_shift': 0
    },
    'Motor2': {
        'in1': 29,
        'in2': 22,
        'spd': 31,
        'adc_channel': 1,
        'target': 180,
        'invert_encoder': False,  # Set to False based on your working Motor 2 code
        'invert_direction': False,  # Set to False unless physically inverted
        'phase_shift': 0
    },
    'Motor3': {
        'in1': 11,
        'in2': 32,
        'spd': 33,
        'adc_channel': 2,
        'target': 180,
        'invert_encoder': False,
        'invert_direction': False,
        'phase_shift': PHASE_SHIFT
    },
    'Motor4': {
        'in1': 12,
        'in2': 13,
        'spd': 35,
        'adc_channel': 3,
        'target': 180,
        'invert_encoder': False,  # Set to False based on your working Motor 2 code
        'invert_direction': False,  # Set to False unless physically inverted
        'phase_shift': PHASE_SHIFT
    }
}

# ----------------------------
# GPIO and SPI Setup
# ----------------------------

# GPIO setup
GPIO.setmode(GPIO.BOARD)
for motor, config in MOTORS_CONFIG.items():
    GPIO.setup(config['in1'], GPIO.OUT)
    GPIO.setup(config['in2'], GPIO.OUT)
    GPIO.setup(config['spd'], GPIO.OUT)

# Set up PWM for motor speed control
pwm_channels = {}
for motor, config in MOTORS_CONFIG.items():
    pwm = GPIO.PWM(config['spd'], 1000)  # 1kHz frequency
    pwm.start(0)
    pwm_channels[motor] = pwm

# Set up MCP3008 ADC
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

# ----------------------------
# Class Definitions
# ----------------------------

class SpikeFilter:
    """
    SpikeFilter class to filter out noise or erroneous readings,
    especially around the dead zone.
    """
    def __init__(self, name, is_inverted=False):
        self.logger = logging.getLogger(f"SpikeFilter-{name}")
        self.filter_active = False
        self.last_valid_reading = None
        self.is_inverted = is_inverted
        self.name = name

    def filter(self, new_value):
        self.logger.debug(f"Raw ADC Value: {new_value}")

        if self.filter_active:
            if self.is_inverted:
                # For inverted encoders, dead zone near high ADC values after inversion
                if new_value < DEAD_ZONE_THRESHOLD_INVERTED:
                    # Valid reading after dead zone
                    self.logger.debug(f"Valid reading after dead zone: {new_value}")
                    self.filter_active = False
                    self.last_valid_reading = new_value
                    return new_value
                else:
                    # Still in dead zone
                    self.logger.debug(f"Still in dead zone: {new_value}")
                    return None
            else:
                # For normal encoders, dead zone near low ADC values
                if new_value > DEAD_ZONE_THRESHOLD_NORMAL:
                    # Valid reading after dead zone
                    self.logger.debug(f"Valid reading after dead zone: {new_value}")
                    self.filter_active = False
                    self.last_valid_reading = new_value
                    return new_value
                else:
                    # Still in dead zone
                    self.logger.debug(f"Still in dead zone: {new_value}")
                    return None
        else:
            if self.last_valid_reading is not None:
                if self.is_inverted:
                    # Inverted encoder: detect sudden rise into dead zone
                    if self.last_valid_reading < DEAD_ZONE_THRESHOLD_INVERTED and new_value >= DEAD_ZONE_THRESHOLD_INVERTED:
                        self.logger.debug(f"Dead zone detected: last valid {self.last_valid_reading}, new {new_value}")
                        self.filter_active = True
                        return None
                else:
                    # Normal encoder: detect sudden drop into dead zone
                    if self.last_valid_reading > ADC_MAX - DEAD_ZONE_THRESHOLD_NORMAL and new_value <= DEAD_ZONE_THRESHOLD_NORMAL:
                        self.logger.debug(f"Dead zone detected: last valid {self.last_valid_reading}, new {new_value}")
                        self.filter_active = True
                        return None
            # Valid reading
            self.last_valid_reading = new_value
            self.logger.debug(f"Valid reading: {new_value}")
            return new_value


class PIDController:
    """
    PID Controller class with integral wind-up protection and output clamping.
    """
    def __init__(self, Kp, Ki, Kd, output_limits=(None, None)):
        self.logger = logging.getLogger("PIDController")
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()
        self.min_output, self.max_output = output_limits

    def compute(self, error):
        current_time = time.time()
        delta_time = current_time - self.last_time
        if delta_time <= 0.0:
            delta_time = 0.0001  # Prevent division by zero

        # Proportional term
        proportional = self.Kp * error

        # Integral term with clamping to prevent wind-up
        self.integral += error * delta_time
        integral = self.Ki * self.integral
        if self.Ki != 0:
            self.integral = max(min(self.integral, 1000 / self.Ki), -1000 / self.Ki)  # Adjust based on Ki

        # Derivative term
        derivative = self.Kd * (error - self.previous_error) / delta_time

        # Compute raw control signal
        control_signal = proportional + integral + derivative

        # Clamp control signal to output limits
        if self.max_output is not None and control_signal > self.max_output:
            control_signal = self.max_output
        elif self.min_output is not None and control_signal < self.min_output:
            control_signal = self.min_output

        # Log the PID terms and control signal
        self.logger.debug(f"Proportional: {proportional:.2f}, Integral: {integral:.2f}, Derivative: {derivative:.2f}, Control Signal (clamped): {control_signal:.2f}")

        # Update for next iteration
        self.previous_error = error
        self.last_time = current_time

        return control_signal

    def reset(self):
        self.logger.debug("PID Controller reset.")
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()


class MotorController:
    """
    MotorController class to handle individual motor operations,
    including reading positions, controlling direction and speed,
    and moving to target positions using PID control.
    """
    def __init__(self, name, in1, in2, pwm, adc_channel, target_position, phase_shift=0,
                 invert_encoder=False, invert_direction=False):
        self.logger = logging.getLogger(name)
        self.name = name
        self.in1 = in1
        self.in2 = in2
        self.pwm = pwm
        self.adc_channel = adc_channel
        self.target_position = target_position
        self.phase_shift = phase_shift
        self.invert_encoder = invert_encoder
        self.invert_direction = invert_direction
        self.spike_filter = SpikeFilter(name, is_inverted=invert_encoder)
        self.pid = PIDController(Kp=0.5, Ki=0.05, Kd=0.02, output_limits=(-100, 100))  # Tuned PID parameters
        self.last_valid_position = None
        self.start_time = None

    def read_position(self):
        """
        Reads the ADC value, applies inversion if necessary, filters the reading,
        and converts it to a positional angle in degrees.
        """
        adc_value = mcp.read_adc(self.adc_channel)
        self.logger.debug(f"Raw ADC Value: {adc_value}")

        # Invert encoder reading if necessary
        if self.invert_encoder:
            raw_value = ADC_MAX - adc_value
            self.logger.debug(f"Inverted ADC Value: {raw_value}")
        else:
            raw_value = adc_value
            self.logger.debug(f"Normal ADC Value: {raw_value}")

        # Apply SpikeFilter
        filtered_value = self.spike_filter.filter(raw_value)

        if filtered_value is None:
            # In dead zone; use last valid position
            if self.last_valid_position is not None:
                self.logger.debug(f"In dead zone. Using last valid position: {self.last_valid_position:.1f}°")
                return self.last_valid_position
            else:
                self.logger.debug("In dead zone with no last valid position. Defaulting to 0°")
                return 0.0

        # Convert ADC value to degrees
        degrees = (filtered_value / ADC_MAX) * MAX_ANGLE
        degrees = (degrees + self.phase_shift) % 360  # Apply phase shift and wrap around

        self.last_valid_position = degrees
        self.logger.debug(f"Converted Position: {degrees:.1f}°")
        return degrees

    def set_motor_direction(self, direction):
        """
        Sets the motor direction based on the control signal.
        """
        # Invert direction if necessary
        if self.invert_direction:
            direction = 'backward' if direction == 'forward' else 'forward'
            self.logger.debug(f"Direction inverted. New direction: {direction}")

        if direction == 'forward':
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
        self.logger.debug(f"Motor direction set to {direction}")

    def move_to_position(self, target):
        """
        Moves the motor towards the target position using PID control.
        Returns True if within tolerance, False otherwise.
        """
        current_position = self.read_position()
        error = target - current_position

        # Normalize error for circular movement (e.g., avoiding large jumps)
        if error > (MAX_ANGLE / 2):
            error -= MAX_ANGLE
        elif error < -(MAX_ANGLE / 2):
            error += MAX_ANGLE

        self.logger.debug(f"Current Position: {current_position:.1f}°, Target: {target:.1f}°, Error: {error:.1f}°")

        # Compute control signal using PID
        control_signal = self.pid.compute(error)

        self.logger.debug(f"Control Signal: {control_signal:.2f}")

        # Determine if within tolerance
        if abs(error) <= 2.0:  # Tolerance of ±2°
            self.stop_motor()
            self.pid.reset()
            self.logger.info("Within tolerance. Stopping motor.")
            return True

        # Determine direction based on control signal
        direction = 'forward' if control_signal > 0 else 'backward'
        self.set_motor_direction(direction)

        # Set motor speed based on control signal
        speed = min(100, max(30, abs(control_signal)))  # Clamp speed between 30% and 100%
        self.pwm.ChangeDutyCycle(speed)
        self.logger.debug(f"Motor speed set to {speed}%")

        return False

    def stop_motor(self):
        """
        Stops the motor by setting both IN1 and IN2 to LOW and setting PWM duty cycle to 0%.
        """
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
        self.logger.debug("Motor stopped.")

    def generate_sawtooth_position(self):
        """
        Generates a sawtooth wave position based on the elapsed time and phase shift.
        """
        if self.start_time is None:
            self.start_time = time.time()

        elapsed_time = time.time() - self.start_time
        position_in_cycle = (elapsed_time % SAWTOOTH_PERIOD) / SAWTOOTH_PERIOD

        # Apply phase shift
        shifted_position = (position_in_cycle * MAX_ANGLE + self.phase_shift) % 360

        # Wrap around to 0° if exceeding MAX_ANGLE
        if shifted_position >= MAX_ANGLE:
            shifted_position = 0.0

        self.logger.debug(f"Generated Sawtooth Position: {shifted_position:.1f}°")
        return shifted_position


# ----------------------------
# Main Execution
# ----------------------------

def main():
    parser = argparse.ArgumentParser(description="Motor control with sawtooth pattern")
    parser.add_argument("motor1_target", type=int, nargs="?", default=180, help="Target position for Motor 1 and Motor 4")
    parser.add_argument("motor3_target", type=int, nargs="?", default=180, help="Target position for Motor 2 and Motor 3")
    args = parser.parse_args()

    # Initialize MotorControllers
    motor_controllers = {}
    try:
        for motor, config in MOTORS_CONFIG.items():
            motor_controllers[motor] = MotorController(
                name=motor,
                in1=config['in1'],
                in2=config['in2'],
                pwm=pwm_channels[motor],
                adc_channel=config['adc_channel'],
                target_position=config.get('target', 180),
                phase_shift=config.get('phase_shift', 0),
                invert_encoder=config.get('invert_encoder', False),
                invert_direction=config.get('invert_direction', False)
            )

        # Update target positions based on command-line arguments
        motor_controllers['Motor1'].target_position = args.motor1_target
        motor_controllers['Motor2'].target_position = args.motor3_target
        motor_controllers['Motor3'].target_position = args.motor3_target
        motor_controllers['Motor4'].target_position = args.motor1_target

        # Initial Calibration
        logging.info("Starting initial calibration...")
        calibration_start = time.time()
        calibration_timeout = 30  # seconds

        while True:
            # Move each motor to its calibration target
            m1_done = motor_controllers['Motor1'].move_to_position(motor_controllers['Motor1'].target_position)
            m2_done = motor_controllers['Motor2'].move_to_position(motor_controllers['Motor2'].target_position)
            m3_done = motor_controllers['Motor3'].move_to_position(motor_controllers['Motor3'].target_position)
            m4_done = motor_controllers['Motor4'].move_to_position(motor_controllers['Motor4'].target_position)

            # Read current positions
            m1_pos = motor_controllers['Motor1'].read_position()
            m2_pos = motor_controllers['Motor2'].read_position()
            m3_pos = motor_controllers['Motor3'].read_position()
            m4_pos = motor_controllers['Motor4'].read_position()

            # Log calibration progress
            logging.info(f"Calibrating - M1: {m1_pos:.1f}° / {motor_controllers['Motor1'].target_position}°, "
                         f"M2: {m2_pos:.1f}° / {motor_controllers['Motor2'].target_position}°, "
                         f"M3: {m3_pos:.1f}° / {motor_controllers['Motor3'].target_position}°, "
                         f"M4: {m4_pos:.1f}° / {motor_controllers['Motor4'].target_position}°")

            # Check if all motors have reached their targets
            if m1_done and m2_done and m3_done and m4_done:
                break

            # Check for calibration timeout
            if time.time() - calibration_start > calibration_timeout:
                logging.error("Calibration timeout! Check motor connections and encoder configurations.")
                raise TimeoutError("Calibration timeout")

            time.sleep(0.1)  # Adjusted sleep for stability

        logging.info("Calibration complete. Stopping motors for 5 seconds...")
        for motor in motor_controllers.values():
            motor.stop_motor()
        time.sleep(5)

        # Start Sawtooth Movement
        logging.info("Starting sawtooth movement pattern...")
        while True:
            # Generate sawtooth positions for each motor
            m1_target = motor_controllers['Motor1'].generate_sawtooth_position()
            m2_target = motor_controllers['Motor2'].generate_sawtooth_position()
            m3_target = motor_controllers['Motor3'].generate_sawtooth_position()
            m4_target = motor_controllers['Motor4'].generate_sawtooth_position()

            # Move motors to generated positions
            m1_done = motor_controllers['Motor1'].move_to_position(m1_target)
            m2_done = motor_controllers['Motor2'].move_to_position(m2_target)
            m3_done = motor_controllers['Motor3'].move_to_position(m3_target)
            m4_done = motor_controllers['Motor4'].move_to_position(m4_target)

            # Read current positions
            m1_pos = motor_controllers['Motor1'].read_position()
            m2_pos = motor_controllers['Motor2'].read_position()
            m3_pos = motor_controllers['Motor3'].read_position()
            m4_pos = motor_controllers['Motor4'].read_position()

            # Log current positions and phase differences
            logging.info(f"Motor 1 - Target: {m1_target:.1f}°, Current: {m1_pos:.1f}°")
            logging.info(f"Motor 2 - Target: {m2_target:.1f}°, Current: {m2_pos:.1f}°")
            logging.info(f"Motor 3 - Target: {m3_target:.1f}°, Current: {m3_pos:.1f}°")
            logging.info(f"Motor 4 - Target: {m4_target:.1f}°, Current: {m4_pos:.1f}°")
            logging.info(f"Phase Difference M1-M3: {abs(m3_pos - m1_pos):.1f}°")
            logging.info(f"Phase Difference M2-M4: {abs(m2_pos - m4_pos):.1f}°")
            logging.info("-" * 60)

            time.sleep(0.1)  # Adjusted sleep for stability

    except KeyboardInterrupt:
        logging.info("Program interrupted by user.")
    except Exception as e:
        logging.exception(f"An error occurred: {e}")
    finally:
        # Stop all motors and clean up GPIO
        for motor in motor_controllers.values():
            motor.stop_motor()
        for pwm in pwm_channels.values():
            pwm.stop()
        GPIO.cleanup()
        logging.info("GPIO cleaned up and motors stopped.")


if __name__ == "__main__":
    main()
