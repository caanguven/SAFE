import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import curses
import sys

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

# Suppress GPIO warnings (optional)
GPIO.setwarnings(False)

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

# Set up MCP3008
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

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
        else:
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

        direction = 'forward' if control_signal > 0 else 'backward'
        self.set_motor_direction(direction)
        speed = min(100, max(30, abs(control_signal)))
        self.pwm.ChangeDutyCycle(speed)
        return False

    def stop_motor(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)

def generate_sawtooth_position(start_time, period=SAWTOOTH_PERIOD, max_angle=MAX_ANGLE):
    elapsed_time = time.time() - start_time
    position_in_cycle = (elapsed_time % period) / period
    position = (position_in_cycle * max_angle) % max_angle
    return position

class MotorGroup:
    def __init__(self, motors, group_phase_difference=0, direction='forward'):
        self.motors = motors
        self.group_phase_difference = group_phase_difference
        self.direction = direction  # 'forward' or 'backward'

    def generate_target_positions(self, base_position):
        target_positions = []
        for motor in self.motors:
            position = (base_position + self.group_phase_difference) % MAX_ANGLE
            if self.direction == 'backward':
                position = (MAX_ANGLE - position) % MAX_ANGLE
            target_positions.append(position)
        return target_positions

def is_opposite_direction(current, new):
    opposites = {
        'forward': 'backward',
        'backward': 'forward',
        'left': 'right',
        'right': 'left'
    }
    return opposites.get(current) == new

def configure_motor_groups(direction, motors):
    if direction == 'forward':
        group1 = MotorGroup(
            motors=[motors['M2'], motors['M3']],
            group_phase_difference=0,
            direction='forward'
        )
        group2 = MotorGroup(
            motors=[motors['M1'], motors['M4']],
            group_phase_difference=180,
            direction='forward'
        )
    elif direction == 'backward':
        group1 = MotorGroup(
            motors=[motors['M2'], motors['M3']],
            group_phase_difference=0,
            direction='backward'
        )
        group2 = MotorGroup(
            motors=[motors['M1'], motors['M4']],
            group_phase_difference=180,
            direction='backward'
        )
    elif direction == 'right':
        group1 = MotorGroup(
            motors=[motors['M1'], motors['M3']],
            group_phase_difference=0,
            direction='backward'
        )
        group2 = MotorGroup(
            motors=[motors['M2'], motors['M4']],
            group_phase_difference=180,
            direction='forward'
        )
    elif direction == 'left':
        group1 = MotorGroup(
            motors=[motors['M1'], motors['M3']],
            group_phase_difference=0,
            direction='forward'
        )
        group2 = MotorGroup(
            motors=[motors['M2'], motors['M4']],
            group_phase_difference=180,
            direction='backward'
        )
    else:
        raise ValueError("Invalid direction")
    return [group1, group2]

def main(stdscr):
    curses.cbreak()
    curses.noecho()
    stdscr.nodelay(True)
    stdscr.keypad(True)

    # Initialize motors
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

    current_direction = 'stable'
    start_time = time.time()
    last_input_time = time.time()
    INPUT_TIMEOUT = 0.5  # Timeout in seconds
    DEBOUNCE_TIME = 0.2  # Debounce interval in seconds
    last_direction_change = time.time()

    # Display instructions
    stdscr.addstr(0, 0, "Motor Control System")
    stdscr.addstr(2, 0, "Controls:")
    stdscr.addstr(3, 2, "Up Arrow    : Forward")
    stdscr.addstr(4, 2, "Down Arrow  : Backward")
    stdscr.addstr(5, 2, "Right Arrow : Turn Right")
    stdscr.addstr(6, 2, "Left Arrow  : Turn Left")
    stdscr.addstr(7, 2, "Space      : Stop")
    stdscr.addstr(8, 2, "Q          : Quit")
    stdscr.refresh()

    try:
        while True:
            key = stdscr.getch()
            current_time = time.time()

            if key != -1:  # If a key was pressed
                # Debounce input
                if current_time - last_direction_change < DEBOUNCE_TIME:
                    # Ignore the input if within debounce interval
                    pass
                else:
                    new_direction = current_direction  # Default to current

                    if key == curses.KEY_UP:
                        new_direction = 'forward'
                    elif key == curses.KEY_DOWN:
                        new_direction = 'backward'
                    elif key == curses.KEY_RIGHT:
                        new_direction = 'right'
                    elif key == curses.KEY_LEFT:
                        new_direction = 'left'
                    elif key == ord(' '):
                        new_direction = 'stable'
                    elif key == ord('q') or key == ord('Q'):
                        break

                    # Check for opposite direction
                    if new_direction == 'stable' or not is_opposite_direction(current_direction, new_direction):
                        if new_direction != current_direction:
                            current_direction = new_direction
                            last_direction_change = current_time
                            last_input_time = current_time
                    else:
                        # Provide feedback that the direction is not allowed
                        stdscr.addstr(9, 0, f"Cannot switch to {new_direction.capitalize()} while moving {current_direction.capitalize()}.")
                        stdscr.clrtoeol()
                        stdscr.refresh()

            elif current_time - last_input_time > INPUT_TIMEOUT:
                # Auto-stop if no input received within timeout period
                if current_direction != 'stable':
                    current_direction = 'stable'

            # Clear previous feedback message
            stdscr.move(9, 0)
            stdscr.clrtoeol()

            # Update motor actions based on current direction
            if current_direction != 'stable':
                motor_groups = configure_motor_groups(current_direction, motors)
                group1, group2 = motor_groups

                base_position = generate_sawtooth_position(start_time)

                group1_targets = group1.generate_target_positions(base_position)
                group2_targets = group2.generate_target_positions(base_position)

                for motor, target in zip(group1.motors, group1_targets):
                    motor.move_to_position(target)

                for motor, target in zip(group2.motors, group2_targets):
                    motor.move_to_position(target)
            else:
                for motor in motors.values():
                    motor.stop_motor()

            # Display current status
            m1_pos = motor1.read_position()
            m2_pos = motor2.read_position()
            m3_pos = motor3.read_position()
            m4_pos = motor4.read_position()

            phase_diff_m1_m3 = abs(m1_pos - m3_pos)
            phase_diff_m2_m4 = abs(m2_pos - m4_pos)

            phase_diff_m1_m3 = min(phase_diff_m1_m3, MAX_ANGLE - phase_diff_m1_m3)
            phase_diff_m2_m4 = min(phase_diff_m2_m4, MAX_ANGLE - phase_diff_m2_m4)

            status_lines = [
                f"Mode: {current_direction.capitalize()}",
                f"M1: {m1_pos:.1f}°, M3: {m3_pos:.1f}° | Phase M1-M3: {phase_diff_m1_m3:.1f}°",
                f"M2: {m2_pos:.1f}°, M4: {m4_pos:.1f}° | Phase M2-M4: {phase_diff_m2_m4:.1f}°"
            ]

            for idx, line in enumerate(status_lines, start=10):
                stdscr.addstr(idx, 0, line)
                stdscr.clrtoeol()

            stdscr.refresh()
            time.sleep(0.02)  # Reduced sleep for better responsiveness

    except Exception as e:
        stdscr.addstr(14, 0, f"Error: {str(e)}")
        stdscr.refresh()
        time.sleep(2)
    finally:
        for motor in motors.values():
            motor.stop_motor()

        motor1_pwm.stop()
        motor2_pwm.stop()
        motor3_pwm.stop()
        motor4_pwm.stop()
        GPIO.cleanup()

        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()
