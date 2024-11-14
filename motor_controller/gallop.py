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

# GPIO Pins for Motor 4 (Please replace with actual pins)
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
        self.target_position = 0  # Will be set by MotorGroup

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
        degrees = (filtered_value / ADC_MAX) * MAX_ANGLE
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
        if error > MAX_ANGLE / 2:
            error -= MAX_ANGLE
        elif error < -MAX_ANGLE / 2:
            error += MAX_ANGLE

        # Use PID to compute control signal
        control_signal = self.pid.compute(error)

        # Determine direction and speed
        if abs(error) <= 2:  # Tolerance
            self.stop_motor()
            return True

        self.set_motor_direction('forward' if control_signal > 0 else 'backward')
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
    def __init__(self, name, motors, phase_difference=0, direction=1):
        """
        :param name: Name of the motor group
        :param motors: List of MotorController instances
        :param phase_difference: Phase difference in degrees relative to the base position
        :param direction: 1 for normal direction, -1 for opposite direction
        """
        self.name = name
        self.motors = motors
        self.phase_difference = phase_difference
        self.direction = direction  # 1 or -1

    def generate_target_positions(self, base_position):
        target_positions = []
        for motor in self.motors:
            # Apply phase difference
            position = (base_position + self.phase_difference) % MAX_ANGLE

            # Apply direction
            if self.direction == -1:
                position = (MAX_ANGLE - position) % MAX_ANGLE

            target_positions.append(position)
        return target_positions

def main():
    parser = argparse.ArgumentParser(description="Motor control with sawtooth pattern and synchronization")
    parser.add_argument("--base_target", type=int, default=90, help="Base target angle in degrees")
    args = parser.parse_args()

    try:
        # Initialize motors with encoder flipping as necessary
        motor1 = MotorController("Motor 1", MOTOR1_IN1, MOTOR1_IN2, motor1_pwm,
                                 MOTOR1_ADC_CHANNEL, encoder_flipped=False)
        motor2 = MotorController("Motor 2", MOTOR2_IN1, MOTOR2_IN2, motor2_pwm,
                                 MOTOR2_ADC_CHANNEL, encoder_flipped=False)
        motor3 = MotorController("Motor 3", MOTOR3_IN1, MOTOR3_IN2, motor3_pwm,
                                 MOTOR3_ADC_CHANNEL, encoder_flipped=False)
        motor4 = MotorController("Motor 4", MOTOR4_IN1, MOTOR4_IN2, motor4_pwm,
                                 MOTOR4_ADC_CHANNEL, encoder_flipped=False)

        # Map motor names to instances for easy reference
        motors = {
            'Motor 1': motor1,
            'Motor 2': motor2,
            'Motor 3': motor3,
            'Motor 4': motor4
        }

        # Define motor groups according to synchronization requirements
        group1 = MotorGroup(
            name="Group 1",
            motors=[motor1, motor2],
            phase_difference=0,    # 0 degrees phase difference
            direction=1            # Forward direction
        )
        group2 = MotorGroup(
            name="Group 2",
            motors=[motor3, motor4],
            phase_difference=180,  # 180 degrees phase difference
            direction=1            # Same forward direction
        )

        motor_groups = [group1, group2]

        # Initial calibration
        print("Starting initial calibration...")
        calibration_start = time.time()
        calibration_timeout = 30  # seconds

        while True:
            calibration_done = True
            for group in motor_groups:
                base_target = args.base_target
                group_targets = group.generate_target_positions(base_target)
                for motor, target in zip(group.motors, group_targets):
                    done = motor.move_to_position(target)
                    calibration_done = calibration_done and done

            # Print calibration progress
            status = []
            for group in motor_groups:
                base_target = args.base_target
                group_targets = group.generate_target_positions(base_target)
                for motor, target in zip(group.motors, group_targets):
                    pos = motor.read_position()
                    status.append(f"{motor.name}: {pos:.1f}° / {target:.1f}°")

            print("\rCalibrating - " + ", ".join(status), end='')

            if calibration_done:
                break

            # Timeout after specified duration
            if time.time() - calibration_start > calibration_timeout:
                print("\nCalibration timeout! Check motor connections.")
                raise TimeoutError("Calibration timeout")

            time.sleep(0.05)

        print("\nCalibration complete. Stopping for 5 seconds...")
        for motor in motors.values():
            motor.stop_motor()
        time.sleep(5)

        print("Starting sawtooth pattern with synchronization...")
        start_time = time.time()
        while True:
            # Generate base position
            base_position = generate_sawtooth_position(start_time)

            # Generate and move target positions for each group
            all_targets = {}
            for group in motor_groups:
                targets = group.generate_target_positions(base_position)
                for motor, target in zip(group.motors, targets):
                    motor.move_to_position(target)
                    all_targets[motor.name] = target

            # Read current positions
            current_positions = {motor.name: motor.read_position() for motor in motors.values()}

            # Display current status
            for motor_name in sorted(current_positions.keys()):
                print(f"{motor_name} - Target: {all_targets[motor_name]:.1f}°, Current: {current_positions[motor_name]:.1f}°")

            # Calculate and display phase differences between groups
            # Assuming Group 1: Motor1 & Motor2, Group 2: Motor3 & Motor4
            phase_diff_group1 = abs(current_positions['Motor 2'] - current_positions['Motor 1'])
            phase_diff_group2 = abs(current_positions['Motor 4'] - current_positions['Motor 3'])
            overall_phase_diff = abs(current_positions['Motor 3'] - current_positions['Motor 1'])

            print(f"Phase Difference M1-M2: {phase_diff_group1:.1f}°, M3-M4: {phase_diff_group2:.1f}°, "
                  f"Overall Phase M1-M3: {overall_phase_diff:.1f}°")
            print("-" * 80)

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        for motor in motors.values():
            motor.stop_motor()
        motor1_pwm.stop()
        motor2_pwm.stop()
        motor3_pwm.stop()
        motor4_pwm.stop()
        GPIO.cleanup()
        print("GPIO cleaned up")

if __name__ == "__main__":
    main()
