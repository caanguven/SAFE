import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import argparse

# Constants for SPI and ADC
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023
DEAD_ZONE_THRESHOLD = 50
SAWTOOTH_PERIOD = 2  # Period in seconds
MIN_ANGLE = 0
MAX_ANGLE = 330

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
        self.name = name  # For debugging purposes

    def filter(self, new_value):
        if self.filter_active:
            # Discard readings in the dead zone (150 to 700)
            if 150 <= new_value <= 700:
                print(f"[{self.name}] Discarding invalid reading during dead zone: {new_value}")
                return None
            else:
                # Valid reading after dead zone
                print(f"[{self.name}] Valid reading after dead zone: {new_value}")
                self.filter_active = False
                self.last_valid_reading = new_value
                return new_value
        else:
            # Not currently filtering
            if self.last_valid_reading is not None and abs(self.last_valid_reading - new_value) > 300:
                # Sudden jump detected, activate filter
                print(f"[{self.name}] Dead zone detected: last valid {self.last_valid_reading}, new {new_value}")
                self.filter_active = True
                return None
            else:
                # Valid reading
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
        self.in_dead_zone = False
        self.last_valid_position = None
        self.pid = PIDController(Kp=0.8, Ki=0.1, Kd=0.05)
        self.encoder_flipped = encoder_flipped
        self.spike_filter = SpikeFilter(name)
        self.movement_direction = 1  # 1 for forward, -1 for backward

    def set_movement_direction(self, movement_direction):
        self.movement_direction = movement_direction

    def read_position(self):
        # Read raw ADC value
        raw_value = mcp.read_adc(self.adc_channel)

        # Invert raw_value if encoder is flipped
        if self.encoder_flipped:
            raw_value = ADC_MAX - raw_value

        # Apply spike filter
        filtered_value = self.spike_filter.filter(raw_value)

        if filtered_value is None:
            # Use last valid position if in dead zone
            return self.last_valid_position if self.last_valid_position is not None else 0

        # Convert filtered ADC value to degrees
        degrees = (filtered_value / ADC_MAX) * 330.0
        self.last_valid_position = degrees
        return degrees

    def set_motor_direction(self, direction):
        if direction == 'forward':
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)

    def move_to_position(self, target):
        current_position = self.read_position()
        error = target - current_position

        # Normalize error for circular movement
        if error > 165:
            error -= 330
        elif error < -165:
            error += 330

        # Use PID to compute control signal
        control_signal = self.pid.compute(error)

        # Adjust control signal based on movement direction
        adjusted_control_signal = control_signal * self.movement_direction

        # Determine direction and speed
        if abs(error) <= 8:  # Tolerance
            self.stop_motor()
            return True

        self.set_motor_direction('forward' if adjusted_control_signal > 0 else 'backward')
        speed = min(100, max(30, abs(adjusted_control_signal)))
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
    def __init__(self, motors, group_phase_difference=0, movement_direction=1):
        """
        motors: list of MotorController instances
        group_phase_difference: in degrees, phase difference relative to base position
        movement_direction: 1 for forward, -1 for backward
        """
        self.motors = motors
        self.group_phase_difference = group_phase_difference
        self.movement_direction = movement_direction
        for motor in self.motors:
            motor.set_movement_direction(self.movement_direction)

    def generate_target_positions(self, base_position):
        target_positions = []
        for motor in self.motors:
            # Adjust the base_position by group phase difference
            position = (base_position + self.group_phase_difference) % MAX_ANGLE
            target_positions.append(position)
        return target_positions

def configure_motor_groups(direction, motors):
    if direction == 'forward':
        group1 = MotorGroup(
            motors=[motors['M2'], motors['M3']],
            group_phase_difference=0,
            movement_direction=1
        )
        group2 = MotorGroup(
            motors=[motors['M1'], motors['M4']],
            group_phase_difference=180,
            movement_direction=1
        )
    elif direction == 'backward':
        group1 = MotorGroup(
            motors=[motors['M2'], motors['M3']],
            group_phase_difference=0,
            movement_direction=-1
        )
        group2 = MotorGroup(
            motors=[motors['M1'], motors['M4']],
            group_phase_difference=180,
            movement_direction=-1
        )
    elif direction == 'left':
        group1 = MotorGroup(
            motors=[motors['M2'], motors['M4']],
            group_phase_difference=0,
            movement_direction=1  # forward
        )
        group2 = MotorGroup(
            motors=[motors['M1'], motors['M3']],
            group_phase_difference=180,
            movement_direction=-1  # backward
        )
    elif direction == 'right':
        group1 = MotorGroup(
            motors=[motors['M2'], motors['M4']],
            group_phase_difference=0,
            movement_direction=-1  # backward
        )
        group2 = MotorGroup(
            motors=[motors['M1'], motors['M3']],
            group_phase_difference=180,
            movement_direction=1  # forward
        )
    else:
        raise ValueError("Invalid direction")
    return [group1, group2]

def calibrate_motors(motor_groups):
    print("\nStarting calibration...")
    calibration_start = time.time()
    base_position = 0  # Starting position for calibration

    # Generate target positions for each group
    group1_targets = motor_groups[0].generate_target_positions(base_position)
    group2_targets = motor_groups[1].generate_target_positions(base_position)

    # Map motors to their targets
    motor_targets = {}
    for motor, target in zip(motor_groups[0].motors, group1_targets):
        motor_targets[motor] = target
    for motor, target in zip(motor_groups[1].motors, group2_targets):
        motor_targets[motor] = target

    while True:
        all_done = True
        for motor, target in motor_targets.items():
            done = motor.move_to_position(target)
            if not done:
                all_done = False

        # Print calibration progress
        positions = []
        for motor in motor_targets.keys():
            positions.append(f"{motor.name}: {motor.read_position():.1f}° -> {motor_targets[motor]:.1f}°")

        print("\rCalibrating - " + ", ".join(positions), end='')

        if all_done:
            break

        # Timeout after 30 seconds
        if time.time() - calibration_start > 30:
            print("\nCalibration timeout! Check motor connections.")
            raise TimeoutError("Calibration timeout")

        time.sleep(0.05)

    print("\nCalibration complete. Proceeding...")

def main():
    parser = argparse.ArgumentParser(description="Motor control with sawtooth pattern and synchronization")
    parser.add_argument("direction", type=str, choices=["forward", "backward", "left", "right"], help="Direction of movement")
    args = parser.parse_args()

    try:
        # Initialize motors with encoder flipping as necessary
        motor1 = MotorController("Motor 1", MOTOR1_IN1, MOTOR1_IN2, motor1_pwm,
                                 MOTOR1_ADC_CHANNEL, encoder_flipped=False)
        motor2 = MotorController("Motor 2", MOTOR2_IN1, MOTOR2_IN2, motor2_pwm,
                                 MOTOR2_ADC_CHANNEL, encoder_flipped=True)
        motor3 = MotorController("Motor 3", MOTOR3_IN1, MOTOR3_IN2, motor3_pwm,
                                 MOTOR3_ADC_CHANNEL, encoder_flipped=False)
        motor4 = MotorController("Motor 4", MOTOR4_IN1, MOTOR4_IN2, motor4_pwm,
                                 MOTOR4_ADC_CHANNEL, encoder_flipped=True)

        # Map motor names to instances
        motors = {
            'M1': motor1,
            'M2': motor2,
            'M3': motor3,
            'M4': motor4
        }

        # Configure motor groups based on direction
        motor_groups = configure_motor_groups(args.direction, motors)

        # Calibrate motors using the same configuration as the movement
        calibrate_motors(motor_groups)
        time.sleep(2)  # Optional pause after calibration

        print("Starting sawtooth pattern with synchronization...")
        start_time = time.time()
        while True:
            # Generate base position
            base_position = generate_sawtooth_position(start_time)

            # Generate target positions for each group
            group1_targets = motor_groups[0].generate_target_positions(base_position)
            group2_targets = motor_groups[1].generate_target_positions(base_position)

            # Move motors in group1
            for motor, target in zip(motor_groups[0].motors, group1_targets):
                motor.move_to_position(target)

            # Move motors in group2
            for motor, target in zip(motor_groups[1].motors, group2_targets):
                motor.move_to_position(target)

            # Read current positions
            m1_pos = motor1.read_position()
            m2_pos = motor2.read_position()
            m3_pos = motor3.read_position()
            m4_pos = motor4.read_position()

            # Calculate phase differences
            phase_diff_m1_m3 = abs(m1_pos - m3_pos)
            phase_diff_m2_m4 = abs(m2_pos - m4_pos)

            # Adjust phase differences to account for circular measurement
            phase_diff_m1_m3 = min(phase_diff_m1_m3, 330 - phase_diff_m1_m3)
            phase_diff_m2_m4 = min(phase_diff_m2_m4, 330 - phase_diff_m2_m4)

            # Print current positions and phase differences
            print(f"\nMotor 1 - Current: {m1_pos:.1f}°")
            print(f"Motor 2 - Current: {m2_pos:.1f}°")
            print(f"Motor 3 - Current: {m3_pos:.1f}°")
            print(f"Motor 4 - Current: {m4_pos:.1f}°")
            print(f"Phase Difference M1-M3: {phase_diff_m1_m3:.1f}°, M2-M4: {phase_diff_m2_m4:.1f}°")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        motor1.stop_motor()
        motor2.stop_motor()
        motor3.stop_motor()
        motor4.stop_motor()
        motor1_pwm.stop()
        motor2_pwm.stop()
        motor3_pwm.stop()
        motor4_pwm.stop()
        GPIO.cleanup()
        print("GPIO cleaned up")

if __name__ == "__main__":
    main()
