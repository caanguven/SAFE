import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import argparse
import busio
import board
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C
import math
import sys
import logging

# Configure basic logging to suppress adafruit debug messages
logging.basicConfig(
    level=logging.ERROR,  # Set to ERROR to suppress DEBUG messages
    format='%(message)s',
    handlers=[logging.StreamHandler()]
)

# Suppress GPIO warnings
GPIO.setwarnings(False)

# Constants for SPI and ADC
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023
MIN_ANGLE = 0
MAX_ANGLE = 330
SAWTOOTH_PERIOD = 2  # Period in seconds

# GPIO Pins configuration (using BCM numbering)
# Motor 1 (Left Front)
MOTOR1_IN1 = 4
MOTOR1_IN2 = 7
MOTOR1_SPD = 24
MOTOR1_ADC_CHANNEL = 0

# Motor 2 (Right Front)
MOTOR2_IN1 = 5
MOTOR2_IN2 = 25
MOTOR2_SPD = 6
MOTOR2_ADC_CHANNEL = 1

# Motor 3 (Left Rear)
MOTOR3_IN1 = 17
MOTOR3_IN2 = 12
MOTOR3_SPD = 13
MOTOR3_ADC_CHANNEL = 2

# Motor 4 (Right Rear)
MOTOR4_IN1 = 18
MOTOR4_IN2 = 27
MOTOR4_SPD = 19
MOTOR4_ADC_CHANNEL = 3

# Initialize MCP3008
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

# IMU setup functions
def quaternion_to_euler(quat_i, quat_j, quat_k, quat_real):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw)
    """
    try:
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (quat_real * quat_i + quat_j * quat_k)
        cosr_cosp = 1 - 2 * (quat_i * quat_i + quat_j * quat_j)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (quat_real * quat_j - quat_k * quat_i)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (quat_real * quat_k + quat_i * quat_j)
        cosy_cosp = 1 - 2 * (quat_j * quat_j + quat_k * quat_k)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Convert to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        return roll_deg, pitch_deg, yaw_deg

    except Exception as e:
        print(f"Error in quaternion conversion: {str(e)}")
        return None, None, None

def setup_imu():
    """Initialize the IMU sensor."""
    try:
        i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
        time.sleep(0.5)
        bno = BNO08X_I2C(i2c)
        time.sleep(1)
        bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
        time.sleep(0.5)
        return bno
    except Exception as e:
        print(f"Failed to initialize IMU: {e}")
        sys.exit(1)

def calibrate_imu(bno):
    """Calibrate IMU to set the initial yaw reference."""
    print("Calibrating IMU... Please keep the robot still.")
    samples = []
    max_samples = 50  # Increased number of samples for better calibration
    for _ in range(max_samples):
        quat = bno.quaternion
        if quat is not None:
            _, _, yaw = quaternion_to_euler(*quat)
            if yaw is not None and not math.isnan(yaw):
                samples.append(yaw)
        time.sleep(0.1)
    if not samples:
        print("Calibration failed: No valid yaw samples collected.")
        sys.exit(1)
    calibration_offset = sum(samples) / len(samples)
    print(f"Calibration complete. Reference yaw: {calibration_offset:.2f}°")
    return calibration_offset

def get_current_yaw(bno, calibration_offset):
    """Get the current yaw angle relative to the calibration offset."""
    try:
        quat = bno.quaternion
        if quat is not None:
            _, _, yaw = quaternion_to_euler(*quat)
            yaw_adjusted = (yaw - calibration_offset + 360) % 360
            if yaw_adjusted > 180:
                yaw_adjusted -= 360  # Convert to range [-180, 180]
            return yaw_adjusted
        else:
            return None
    except KeyError:
        # Handle unknown report type gracefully
        print("Warning: Received unknown report type from IMU. Ignoring packet.")
        return None
    except Exception as e:
        print(f"Error reading yaw: {e}")
        return None

class SpikeFilter:
    def __init__(self, name):
        self.filter_active = False
        self.last_valid_reading = None
        self.name = name

    def filter(self, new_value):
        if self.filter_active:
            if 150 <= new_value <= 700:
                return None
            else:
                self.filter_active = False
                self.last_valid_reading = new_value
                return new_value
        else:
            if self.last_valid_reading is not None and abs(self.last_valid_reading - new_value) > 300:
                self.filter_active = True
                return None
            else:
                self.last_valid_reading = new_value
                return new_value

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()

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

        return control_signal

class MotorController:
    def __init__(self, name, in1, in2, pwm, adc_channel, encoder_flipped=False):
        self.name = name
        self.in1 = in1
        self.in2 = in2
        self.pwm = pwm
        self.adc_channel = adc_channel
        self.position = 0
        self.last_valid_position = None
        self.pid = PIDController(Kp=0.8, Ki=0.1, Kd=0.05)
        self.encoder_flipped = encoder_flipped
        self.spike_filter = SpikeFilter(name)

    def read_position(self):
        raw_value = mcp.read_adc(self.adc_channel)

        if self.encoder_flipped:
            raw_value = ADC_MAX - raw_value

        filtered_value = self.spike_filter.filter(raw_value)

        if filtered_value is None:
            return self.last_valid_position if self.last_valid_position is not None else 0

        degrees = (filtered_value / ADC_MAX) * 330.0
        self.last_valid_position = degrees
        return degrees

    def set_motor_direction(self, direction):
        if direction == 'forward':
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        elif direction == 'backward':
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
        else:  # stop
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.LOW)

    def move_to_position(self, target):
        current_position = self.read_position()
        error = target - current_position

        if error > 165:
            error -= 330
        elif error < -165:
            error += 330

        control_signal = self.pid.compute(error)

        if abs(error) <= 2:
            self.stop_motor()
            return True

        self.set_motor_direction('forward' if control_signal > 0 else 'backward')
        speed = min(100, max(30, abs(control_signal)))
        self.pwm.ChangeDutyCycle(speed)
        return False

    def stop_motor(self):
        self.set_motor_direction('stop')
        self.pwm.ChangeDutyCycle(0)

def setup_motors():
    """Initialize GPIO pins and PWM for motors."""
    motor_pins = [
        (MOTOR1_IN1, MOTOR1_IN2, MOTOR1_SPD),
        (MOTOR2_IN1, MOTOR2_IN2, MOTOR2_SPD),
        (MOTOR3_IN1, MOTOR3_IN2, MOTOR3_SPD),
        (MOTOR4_IN1, MOTOR4_IN2, MOTOR4_SPD)
    ]
    
    for in1, in2, spd in motor_pins:
        GPIO.setup(in1, GPIO.OUT)
        GPIO.setup(in2, GPIO.OUT)
        GPIO.setup(spd, GPIO.OUT)
    
    motor_pwms = {}
    for i, (in1, in2, spd) in enumerate(motor_pins, start=1):
        pwm = GPIO.PWM(spd, 1000)
        pwm.start(0)
        motor_pwms[f"M{i}"] = pwm
    
    return motor_pins, motor_pwms

def generate_sawtooth_position(start_time, period=SAWTOOTH_PERIOD, max_angle=MAX_ANGLE):
    """Generate a sawtooth wave position for gait."""
    elapsed_time = time.time() - start_time
    position_in_cycle = (elapsed_time % period) / period
    position = (position_in_cycle * max_angle) % max_angle
    return position

class MotorGroup:
    def __init__(self, motors, group_phase_difference=0, direction=1):
        self.motors = motors
        self.group_phase_difference = group_phase_difference
        self.direction = direction

    def generate_target_positions(self, base_position, phase_offsets=None):
        """Generate target positions for each motor in the group."""
        target_positions = []
        for motor in self.motors:
            position = (base_position + self.group_phase_difference) % MAX_ANGLE
            if self.direction == -1:
                position = (MAX_ANGLE - position) % MAX_ANGLE
            # Apply individual phase offset if provided
            if phase_offsets and motor.name in phase_offsets:
                position = (position + phase_offsets[motor.name]) % MAX_ANGLE
            target_positions.append(position)
        return target_positions

def configure_motor_groups(motors):
    """Configure motor groups for gait."""
    group1 = MotorGroup(
        motors=[motors['M2'], motors['M3']],
        group_phase_difference=0,
        direction=1
    )
    group2 = MotorGroup(
        motors=[motors['M1'], motors['M4']],
        group_phase_difference=180,
        direction=1
    )
    return [group1, group2]

def stop_all_motors(motors):
    """Stop all motors."""
    for motor in motors.values():
        motor.stop_motor()

def perform_point_turn(motors, motor_pins, turn_direction, angle, bno, calibration_offset):
    """
    Perform a point turn to correct the robot's direction.

    :param motors: Dictionary of MotorController instances.
    :param motor_pins: List of motor pin tuples.
    :param turn_direction: 'left' or 'right'.
    :param angle: Angle to turn in degrees.
    :param bno: IMU sensor object.
    :param calibration_offset: Initial yaw offset.
    """
    motor_speed = 70  # Speed for point turn; adjust as needed

    # Configure motors for point turn
    if turn_direction == 'right':
        # Right point turn: Left motors forward, Right motors backward
        motors['M1'].set_motor_direction('forward')
        motors['M2'].set_motor_direction('backward')
        motors['M3'].set_motor_direction('forward')
        motors['M4'].set_motor_direction('backward')
    else:
        # Left point turn: Left motors backward, Right motors forward
        motors['M1'].set_motor_direction('backward')
        motors['M2'].set_motor_direction('forward')
        motors['M3'].set_motor_direction('backward')
        motors['M4'].set_motor_direction('forward')

    # Set motor speeds
    for motor in motors.values():
        motor.set_motor_speed(motor_speed)

    print(f"\nStarting {turn_direction} point turn of {angle}°")

    # Record initial yaw
    initial_yaw = get_current_yaw(bno, calibration_offset)
    if initial_yaw is None:
        print("Failed to get initial yaw for point turn.")
        return

    # Determine target yaw
    if turn_direction == 'right':
        target_yaw = (initial_yaw + angle) % 360
    else:
        target_yaw = (initial_yaw - angle) % 360

    if target_yaw > 180:
        target_yaw -= 360  # Convert to range [-180, 180]

    # Main turn loop
    while True:
        current_yaw = get_current_yaw(bno, calibration_offset)
        if current_yaw is None:
            print("Yaw reading failed during point turn.")
            break

        # Calculate turned angle
        if turn_direction == 'right':
            turned_angle = (current_yaw - initial_yaw) % 360
            if turned_angle > 180:
                turned_angle -= 360
        else:
            turned_angle = (initial_yaw - current_yaw) % 360
            if turned_angle > 180:
                turned_angle -= 360

        print(f"Point Turn - Current yaw: {current_yaw:.2f}°, Turned: {turned_angle:.2f}°    ", end='\r')

        if abs(turned_angle) >= angle:
            print(f"\nPoint turn completed! Final yaw: {current_yaw:.2f}°")
            break

        time.sleep(0.05)

    # Stop motors after point turn
    stop_all_motors(motors)
    time.sleep(0.5)  # Brief pause after turn

def set_motor_speed(motor, speed):
    """Set the speed of a specified motor."""
    motor.pwm.ChangeDutyCycle(speed)

def main():
    parser = argparse.ArgumentParser(description='Robot Movement Script with IMU Feedback')
    args = parser.parse_args()

    # Initialize motors
    motor_pins, motor_pwms = setup_motors()

    # Initialize IMU
    bno = setup_imu()
    calibration_offset = calibrate_imu(bno)

    # Thresholds for yaw deviation (degrees)
    YAW_THRESHOLD = 10.0
    # Point turn correction angle (degrees)
    CORRECTION_ANGLE = 10.0  # Adjust as needed for desired correction

    # Initialize motor controllers
    motor1 = MotorController("M1", MOTOR1_IN1, MOTOR1_IN2, motor_pwms['M1'], MOTOR1_ADC_CHANNEL, encoder_flipped=False)
    motor2 = MotorController("M2", MOTOR2_IN1, MOTOR2_IN2, motor_pwms['M2'], MOTOR2_ADC_CHANNEL, encoder_flipped=True)
    motor3 = MotorController("M3", MOTOR3_IN1, MOTOR3_IN2, motor_pwms['M3'], MOTOR3_ADC_CHANNEL, encoder_flipped=False)
    motor4 = MotorController("M4", MOTOR4_IN1, MOTOR4_IN2, motor_pwms['M4'], MOTOR4_ADC_CHANNEL, encoder_flipped=True)

    motors = {
        'M1': motor1,
        'M2': motor2,
        'M3': motor3,
        'M4': motor4
    }

    # Start time for gait generation
    start_time = time.time()

    try:
        while True:
            # Read current yaw angle
            current_yaw = get_current_yaw(bno, calibration_offset)
            if current_yaw is None:
                print("Yaw reading failed. Skipping correction.")
                correction = 0.0
            else:
                # Determine corrective action based on yaw deviation
                if current_yaw > YAW_THRESHOLD:
                    # Robot is turning left unintentionally; perform corrective right point turn
                    correction_direction = 'right'
                    print(f"Yaw deviation: {current_yaw:.2f}°, performing corrective right point turn")
                    perform_point_turn(motors, motor_pins, correction_direction, CORRECTION_ANGLE, bno, calibration_offset)
                elif current_yaw < -YAW_THRESHOLD:
                    # Robot is turning right unintentionally; perform corrective left point turn
                    correction_direction = 'left'
                    print(f"Yaw deviation: {current_yaw:.2f}°, performing corrective left point turn")
                    perform_point_turn(motors, motor_pins, correction_direction, CORRECTION_ANGLE, bno, calibration_offset)
                else:
                    print(f"Yaw deviation: {current_yaw:.2f}°, moving straight")

            # Generate target positions for gait
            base_position = generate_sawtooth_position(start_time)

            # No phase offsets needed during straight movement
            phase_offsets = {}

            # Generate motor target positions
            motor_groups = configure_motor_groups(motors)
            group1, group2 = motor_groups

            group1_targets = group1.generate_target_positions(base_position, phase_offsets)
            group2_targets = group2.generate_target_positions(base_position, phase_offsets)

            # Move motors to target positions
            for motor, target in zip(group1.motors, group1_targets):
                motor.move_to_position(target)

            for motor, target in zip(group2.motors, group2_targets):
                motor.move_to_position(target)

            # Sleep for a short time to prevent excessive CPU usage
            time.sleep(0.02)

    except KeyboardInterrupt:
        # Allow the user to stop the script with Ctrl+C
        print("\nOperation cancelled by user.")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
    finally:
        stop_all_motors(motors)
        for pwm in motor_pwms.values():
            pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
