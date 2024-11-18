import curses
import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import math
import busio
import board
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C
import threading

# Constants for SPI and ADC
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023
MIN_ANGLE = 0
MAX_ANGLE = 330
SAWTOOTH_PERIOD = 2  # Period in seconds

# GPIO Pins for Motor 1
MOTOR1_IN1 = 7
MOTOR1_IN2 = 26
MOTOR1_SPD = 18
MOTOR1_ADC_CHANNEL = 0

# GPIO Pins for Motor 2
MOTOR2_IN1 = 29
MOTOR2_IN2 = 22
MOTOR2_SPD = 31
MOTOR2_ADC_CHANNEL = 1

# GPIO Pins for Motor 3
MOTOR3_IN1 = 11
MOTOR3_IN2 = 32
MOTOR3_SPD = 33
MOTOR3_ADC_CHANNEL = 2

# GPIO Pins for Motor 4
MOTOR4_IN1 = 12
MOTOR4_IN2 = 13
MOTOR4_SPD = 35
MOTOR4_ADC_CHANNEL = 3

# Initialize MCP3008
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

# GPIO setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(MOTOR1_IN1, GPIO.OUT)
GPIO.setup(MOTOR1_IN2, GPIO.OUT)
GPIO.setup(MOTOR1_SPD, GPIO.OUT)
GPIO.setup(MOTOR2_IN1, GPIO.OUT)
GPIO.setup(MOTOR2_IN2, GPIO.OUT)
GPIO.setup(MOTOR2_SPD, GPIO.OUT)
GPIO.setup(MOTOR3_IN1, GPIO.OUT)
GPIO.setup(MOTOR3_IN2, GPIO.OUT)
GPIO.setup(MOTOR3_SPD, GPIO.OUT)
GPIO.setup(MOTOR4_IN1, GPIO.OUT)
GPIO.setup(MOTOR4_IN2, GPIO.OUT)
GPIO.setup(MOTOR4_SPD, GPIO.OUT)

# Set up PWM for motor speed control
motor1_pwm = GPIO.PWM(MOTOR1_SPD, 1000)
motor1_pwm.start(0)
motor2_pwm = GPIO.PWM(MOTOR2_SPD, 1000)
motor2_pwm.start(0)
motor3_pwm = GPIO.PWM(MOTOR3_SPD, 1000)
motor3_pwm.start(0)
motor4_pwm = GPIO.PWM(MOTOR4_SPD, 1000)
motor4_pwm.start(0)

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
            delta_time = 0.0001  # Prevent division by zero

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
        self.encoder_flipped = encoder_flipped
        self.position = 0
        self.last_valid_position = None
        self.pid = PIDController(Kp=0.8, Ki=0.1, Kd=0.05)
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

    def set_motor_speed(self, speed):
        """Set the speed of the motor using PWM."""
        self.pwm.ChangeDutyCycle(speed)

    def move_to_position(self, target):
        current_position = self.read_position()
        error = target - current_position

        # Normalize error to range [-180, 180]
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        control_signal = self.pid.compute(error)

        if abs(error) <= 2:
            self.stop_motor()
            return True

        self.set_motor_direction('forward' if control_signal > 0 else 'backward')
        speed = min(100, max(30, abs(control_signal)))
        self.set_motor_speed(speed)
        return False

    def stop_motor(self):
        self.set_motor_direction('stop')
        self.set_motor_speed(0)

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
        return None

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
        return 0.0
    calibration_offset = sum(samples) / len(samples)
    print(f"Calibration complete. Reference yaw: {calibration_offset:.2f}°")
    return calibration_offset

def get_current_yaw(bno, calibration_offset):
    """Get the current yaw angle relative to the calibration offset."""
    try:
        quat = bno.quaternion
        if quat is not None:
            _, _, yaw = quaternion_to_euler(*quat)
            yaw_adjusted = (yaw - calibration_offset) % 360
            if yaw_adjusted > 180:
                yaw_adjusted -= 360  # Convert to range [-180, 180]
            return yaw_adjusted
        else:
            return None
    except Exception as e:
        print(f"Error reading yaw: {e}")
        return None

class GaitController:
    def __init__(self, motors):
        self.motors = motors
        self.start_time = time.time()

    def generate_sawtooth_position(self, period=SAWTOOTH_PERIOD, max_angle=MAX_ANGLE):
        """Generate a sawtooth wave position for gait."""
        elapsed_time = time.time() - self.start_time
        position_in_cycle = (elapsed_time % period) / period
        position = (position_in_cycle * max_angle) % max_angle
        return position

    def update_gait(self, direction):
        """Update motor positions based on gait and direction."""
        base_position = self.generate_sawtooth_position()
        
        # Define motor groups based on direction
        motor_groups = configure_motor_groups(direction, self.motors)
        for group in motor_groups:
            target_positions = group.generate_target_positions(base_position)
            for motor, target in zip(group.motors, target_positions):
                motor.move_to_position(target)

class MotorGroup:
    def __init__(self, motors, group_phase_difference=0, direction=1):
        self.motors = motors
        self.group_phase_difference = group_phase_difference
        self.direction = direction

    def generate_target_positions(self, base_position):
        target_positions = []
        for motor in self.motors:
            position = (base_position + self.group_phase_difference) % MAX_ANGLE
            if self.direction == -1:
                position = (MAX_ANGLE - position) % MAX_ANGLE
            target_positions.append(position)
        return target_positions

def configure_motor_groups(direction, motors):
    """Configure motor groups based on movement direction."""
    if direction == 'forward':
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
    elif direction == 'backward':
        group1 = MotorGroup(
            motors=[motors['M2'], motors['M3']],
            group_phase_difference=0,
            direction=-1
        )
        group2 = MotorGroup(
            motors=[motors['M1'], motors['M4']],
            group_phase_difference=180,
            direction=-1
        )
    elif direction == 'right':
        group1 = MotorGroup(
            motors=[motors['M1'], motors['M3']],
            group_phase_difference=0,
            direction=-1
        )
        group2 = MotorGroup(
            motors=[motors['M2'], motors['M4']],
            group_phase_difference=180,
            direction=1
        )
    elif direction == 'left':
        group1 = MotorGroup(
            motors=[motors['M1'], motors['M3']],
            group_phase_difference=0,
            direction=1
        )
        group2 = MotorGroup(
            motors=[motors['M2'], motors['M4']],
            group_phase_difference=180,
            direction=-1
        )
    else:
        group1 = MotorGroup(
            motors=[motors['M1'], motors['M2'], motors['M3'], motors['M4']],
            group_phase_difference=0,
            direction=0
        )
        group2 = MotorGroup(
            motors=[],
            group_phase_difference=0,
            direction=0
        )
    return [group1, group2]

def perform_point_turn(motors, turn_direction, angle, bno, calibration_offset):
    """
    Perform a point turn to correct the robot's direction.

    :param motors: Dictionary of MotorController instances.
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
    elif turn_direction == 'left':
        # Left point turn: Left motors backward, Right motors forward
        motors['M1'].set_motor_direction('backward')
        motors['M2'].set_motor_direction('forward')
        motors['M3'].set_motor_direction('backward')
        motors['M4'].set_motor_direction('forward')
    else:
        print("Invalid turn direction specified.")
        return

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
    for motor in motors.values():
        motor.stop_motor()
    time.sleep(0.5)  # Brief pause after turn

def monitor_yaw(motors, bno, calibration_offset, stop_event):
    """Continuously monitor yaw and perform corrective turns if needed."""
    YAW_THRESHOLD = 10.0  # degrees
    CORRECTION_ANGLE = 10.0  # degrees

    while not stop_event.is_set():
        current_yaw = get_current_yaw(bno, calibration_offset)
        if current_yaw is None:
            print("Yaw reading failed. Skipping correction.")
            time.sleep(0.5)
            continue

        if current_yaw > YAW_THRESHOLD:
            print(f"\nYaw deviation: {current_yaw:.2f}°, performing corrective right point turn")
            perform_point_turn(motors, 'right', CORRECTION_ANGLE, bno, calibration_offset)
        elif current_yaw < -YAW_THRESHOLD:
            print(f"\nYaw deviation: {current_yaw:.2f}°, performing corrective left point turn")
            perform_point_turn(motors, 'left', CORRECTION_ANGLE, bno, calibration_offset)
        else:
            # Yaw is within acceptable range; continue normal operation
            pass

        time.sleep(0.5)  # Adjust the frequency of yaw checks as needed

def main(stdscr):
    # Initialize curses
    curses.cbreak()
    curses.noecho()
    stdscr.nodelay(True)
    stdscr.keypad(True)

    # Initialize MotorController instances
    motor1 = MotorController("M1", MOTOR1_IN1, MOTOR1_IN2, motor1_pwm,
                            MOTOR1_ADC_CHANNEL, encoder_flipped=False)
    motor2 = MotorController("M2", MOTOR2_IN1, MOTOR2_IN2, motor2_pwm,
                            MOTOR2_ADC_CHANNEL, encoder_flipped=True)
    motor3 = MotorController("M3", MOTOR3_IN1, MOTOR3_IN2, motor3_pwm,
                            MOTOR3_ADC_CHANNEL, encoder_flipped=False)
    motor4 = MotorController("M4", MOTOR4_IN1, MOTOR4_IN2, motor4_pwm,
                            MOTOR4_ADC_CHANNEL, encoder_flipped=True)

    motors = {
        'M1': motor1,
        'M2': motor2,
        'M3': motor3,
        'M4': motor4
    }

    # Initialize IMU
    bno = setup_imu()
    if bno is None:
        stdscr.addstr(12, 0, "IMU Initialization Failed. Exiting...")
        stdscr.refresh()
        time.sleep(2)
        return

    # Calibrate IMU
    calibration_offset = calibrate_imu(bno)

    # Initialize GaitController
    gait_controller = GaitController(motors)

    # Start IMU monitoring thread
    stop_event = threading.Event()
    yaw_thread = threading.Thread(target=monitor_yaw, args=(motors, bno, calibration_offset, stop_event))
    yaw_thread.start()

    # Display instructions
    stdscr.addstr(0, 0, "Motor Control System with Gait and IMU-Based Corrections")
    stdscr.addstr(2, 0, "Controls:")
    stdscr.addstr(3, 2, "Up Arrow    : Forward")
    stdscr.addstr(4, 2, "Down Arrow  : Backward")
    stdscr.addstr(5, 2, "Right Arrow : Turn Right")
    stdscr.addstr(6, 2, "Left Arrow  : Turn Left")
    stdscr.addstr(7, 2, "Space       : Stop")
    stdscr.addstr(8, 2, "Q           : Quit")
    stdscr.refresh()

    current_direction = 'stable'

    try:
        while True:
            key = stdscr.getch()

            if key != -1:
                if key == curses.KEY_UP:
                    current_direction = 'forward'
                elif key == curses.KEY_DOWN:
                    current_direction = 'backward'
                elif key == curses.KEY_RIGHT:
                    current_direction = 'right'
                elif key == curses.KEY_LEFT:
                    current_direction = 'left'
                elif key == ord(' '):
                    current_direction = 'stable'
                elif key in [ord('q'), ord('Q')]:
                    break

            # Update gait based on current direction
            gait_controller.update_gait(current_direction)

            # Display current status
            stdscr.move(10, 0)
            stdscr.clrtoeol()
            stdscr.addstr(10, 0, f"Current Direction: {current_direction.capitalize()}")

            stdscr.move(12, 0)
            stdscr.clrtoeol()
            stdscr.addstr(12, 0, "Press 'Q' to quit.")

            stdscr.refresh()
            time.sleep(0.02)

    except Exception as e:
        stdscr.addstr(14, 0, f"Error: {str(e)}")
        stdscr.refresh()
        time.sleep(2)
    finally:
        # Stop IMU monitoring thread
        stop_event.set()
        yaw_thread.join()

        # Stop all motors
        for motor in motors.values():
            motor.stop_motor()

        # Stop PWM
        motor1_pwm.stop()
        motor2_pwm.stop()
        motor3_pwm.stop()
        motor4_pwm.stop()

        GPIO.cleanup()

        # Restore curses settings
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()

if __name__ == "__main__":
    curses.wrapper(main)
