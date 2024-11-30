import warnings
import argparse
import sys
import logging
import math
import time
import signal
import os

import RPi.GPIO as GPIO
import busio
import board
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import threading

# Suppress specific warnings
warnings.filterwarnings("ignore", category=RuntimeWarning, module="adafruit_blinka.microcontroller.generic_linux.i2c")
warnings.filterwarnings("ignore", category=RuntimeWarning)

# Configure basic logging
logging.basicConfig(
    level=logging.INFO,
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

    def reset(self):
        self.filter_active = False
        self.last_valid_reading = None

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

        return control_signal

    def reset(self):
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()

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

    def read_position(self, mcp):
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
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)

    def move_to_position(self, target, mcp):
        try:
            current_position = self.read_position(mcp)
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
        except Exception as e:
            self.reset()
            return False

    def stop_motor(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)

    def reset(self):
        self.stop_motor()
        self.pid.reset()
        self.spike_filter.reset()

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

class MotorControlSystem:
    def __init__(self, mcp, mode='walk'):
        self.mode = mode  # 'walk', 'trot', or 'gallop'
        self.current_direction = 'forward'
        self.lock = threading.Lock()
        self.running = True
        self.mcp = mcp

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
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
        self.motor1_pwm = GPIO.PWM(MOTOR1_SPD, 1000)
        self.motor1_pwm.start(0)
        self.motor2_pwm = GPIO.PWM(MOTOR2_SPD, 1000)
        self.motor2_pwm.start(0)
        self.motor3_pwm = GPIO.PWM(MOTOR3_SPD, 1000)
        self.motor3_pwm.start(0)
        self.motor4_pwm = GPIO.PWM(MOTOR4_SPD, 1000)
        self.motor4_pwm.start(0)

        # Initialize motors
        self.motor1 = MotorController("M1", MOTOR1_IN1, MOTOR1_IN2, self.motor1_pwm,
                                      MOTOR1_ADC_CHANNEL, encoder_flipped=False)
        self.motor2 = MotorController("M2", MOTOR2_IN1, MOTOR2_IN2, self.motor2_pwm,
                                      MOTOR2_ADC_CHANNEL, encoder_flipped=True)
        self.motor3 = MotorController("M3", MOTOR3_IN1, MOTOR3_IN2, self.motor3_pwm,
                                      MOTOR3_ADC_CHANNEL, encoder_flipped=False)
        self.motor4 = MotorController("M4", MOTOR4_IN1, MOTOR4_IN2, self.motor4_pwm,
                                      MOTOR4_ADC_CHANNEL, encoder_flipped=True)

        self.motors = {
            'M1': self.motor1,
            'M2': self.motor2,
            'M3': self.motor3,
            'M4': self.motor4
        }

        self.start_time = time.time()

        # Start the control loop in a separate thread
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()

    def set_mode(self, mode):
        with self.lock:
            self.mode = mode

    def generate_sawtooth_position(self):
        elapsed_time = time.time() - self.start_time
        position_in_cycle = (elapsed_time % SAWTOOTH_PERIOD) / SAWTOOTH_PERIOD
        position = (position_in_cycle * MAX_ANGLE) % MAX_ANGLE
        return position

    def configure_motor_groups(self, direction, mode):
        if mode == 'gallop' and direction in ['forward', 'backward']:
            # Gallop gait
            direction_value = 1 if direction == 'forward' else -1
            group1 = MotorGroup(
                motors=[self.motors['M2'], self.motors['M1']],
                group_phase_difference=0,
                direction=direction_value
            )
            group2 = MotorGroup(
                motors=[self.motors['M4'], self.motors['M3']],
                group_phase_difference=180,
                direction=direction_value
            )
            return [group1, group2]
        elif mode == 'trot' and direction in ['forward', 'backward']:
            # Trot gait (similar to normal gait)
            direction_value = 1 if direction == 'forward' else -1
            group1 = MotorGroup(
                motors=[self.motors['M2'], self.motors['M3']],
                group_phase_difference=0,
                direction=direction_value
            )
            group2 = MotorGroup(
                motors=[self.motors['M1'], self.motors['M4']],
                group_phase_difference=180,
                direction=direction_value
            )
            return [group1, group2]
        elif mode == 'walk' and direction in ['forward', 'backward']:
            # Walk gait
            direction_value = 1 if direction == 'forward' else -1
            group1 = MotorGroup(
                motors=[self.motors['M2']],
                group_phase_difference=0,
                direction=direction_value
            )
            group2 = MotorGroup(
                motors=[self.motors['M4']],
                group_phase_difference=90,
                direction=direction_value
            )
            group3 = MotorGroup(
                motors=[self.motors['M1']],
                group_phase_difference=180,
                direction=direction_value
            )
            group4 = MotorGroup(
                motors=[self.motors['M3']],
                group_phase_difference=270,
                direction=direction_value
            )
            return [group1, group2, group3, group4]
        else:
            # Default gait
            direction_value = 1 if direction == 'forward' else -1
            group1 = MotorGroup(
                motors=[self.motors['M2'], self.motors['M3']],
                group_phase_difference=0,
                direction=direction_value
            )
            group2 = MotorGroup(
                motors=[self.motors['M1'], self.motors['M4']],
                group_phase_difference=180,
                direction=direction_value
            )
            return [group1, group2]

    def control_loop(self):
        while self.running:
            with self.lock:
                direction = self.current_direction
                mode = self.mode

            motor_groups = self.configure_motor_groups(direction, mode)
            base_position = self.generate_sawtooth_position()

            for group in motor_groups:
                targets = group.generate_target_positions(base_position)
                for motor, target in zip(group.motors, targets):
                    motor.move_to_position(target, self.mcp)

            time.sleep(0.02)  # 20 ms delay

    def stop(self):
        self.running = False
        self.control_thread.join()
        for motor in self.motors.values():
            motor.stop_motor()

        self.motor1_pwm.stop()
        self.motor2_pwm.stop()
        self.motor3_pwm.stop()
        self.motor4_pwm.stop()
        GPIO.cleanup()

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
    """Initialize IMU with retry mechanism"""
    max_retries = 3
    retry_delay = 2  # seconds

    for attempt in range(max_retries):
        try:
            print(f"Attempting to initialize IMU (attempt {attempt + 1}/{max_retries})...")

            i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
            time.sleep(0.5)

            bno = BNO08X_I2C(i2c)
            time.sleep(1)

            bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
            time.sleep(0.5)

            # Verify sensor
            quat = bno.quaternion
            if quat is None:
                raise RuntimeError("Unable to read quaternion data")

            # Test quaternion conversion
            roll, pitch, yaw = quaternion_to_euler(*quat)
            if any(x is None for x in [roll, pitch, yaw]):
                raise RuntimeError("Invalid quaternion conversion")

            print("IMU initialized successfully")
            return bno

        except Exception as e:
            print(f"Attempt {attempt + 1} failed: {str(e)}")
            if attempt < max_retries - 1:
                print(f"Retrying in {retry_delay} seconds...")
                time.sleep(retry_delay)
            else:
                print("IMU initialization failed after all attempts")
                raise

def calibrate_imu(bno):
    """Calibrate IMU with improved error handling"""
    print("\nCalibrating IMU (stay still)...")
    samples = []
    max_samples = 50
    timeout = 30  # seconds
    start_time = time.time()

    while len(samples) < max_samples and time.time() - start_time < timeout:
        try:
            if len(samples) % 10 == 0 and len(samples) != 0:
                print(f"Collecting samples... {len(samples)}/{max_samples}", end='\r')
            quat = bno.quaternion

            if quat is not None and len(quat) == 4:
                roll, pitch, yaw = quaternion_to_euler(*quat)
                if None not in (roll, pitch, yaw) and not any(math.isnan(x) for x in [roll, pitch, yaw]):
                    samples.append((roll, pitch, yaw))
            else:
                print("\nInvalid quaternion reading, retrying...")
                time.sleep(0.5)

        except Exception as e:
            print(f"\nError during calibration: {str(e)}")
            time.sleep(0.5)

    if len(samples) >= max_samples / 2:
        # Compute average offsets for roll, pitch, yaw
        sum_roll = sum(sample[0] for sample in samples)
        sum_pitch = sum(sample[1] for sample in samples)
        sum_yaw = sum(sample[2] for sample in samples)

        calibration_offset = {
            'roll': sum_roll / len(samples),
            'pitch': sum_pitch / len(samples),
            'yaw': sum_yaw / len(samples)
        }

        print(f"\nCalibration complete. Reference angles:")
        print(f"Roll offset: {calibration_offset['roll']:.2f}°")
        print(f"Pitch offset: {calibration_offset['pitch']:.2f}°")
        print(f"Yaw offset: {calibration_offset['yaw']:.2f}°")
        return calibration_offset
    else:
        raise RuntimeError(f"Failed to collect enough valid samples (got {len(samples)} of {max_samples})")

def get_current_euler(bno, calibration_offsets, retries=3):
    """Get current Euler angles with calibration offsets"""
    for _ in range(retries):
        try:
            quat = bno.quaternion
            if quat is not None and len(quat) == 4 and not any(math.isnan(x) for x in quat):
                roll, pitch, yaw = quaternion_to_euler(*quat)
                if None not in (roll, pitch, yaw):
                    # Apply calibration offsets
                    roll_adjusted = roll - calibration_offsets['roll']
                    pitch_adjusted = pitch - calibration_offsets['pitch']
                    yaw_adjusted = yaw - calibration_offsets['yaw']

                    # Normalize angles to [-180, 180]
                    roll_adjusted = (roll_adjusted + 180) % 360 - 180
                    pitch_adjusted = (pitch_adjusted + 180) % 360 - 180
                    yaw_adjusted = (yaw_adjusted + 180) % 360 - 180

                    return roll_adjusted, pitch_adjusted, yaw_adjusted
        except Exception:
            time.sleep(0.1)
    return None, None, None

def main():
    parser = argparse.ArgumentParser(description='Quadruped Robot Controller with Gait Modes')
    parser.add_argument('--mode', type=str, choices=['walk', 'trot', 'gallop'], default='walk',
                        help='Select the gait mode for the robot')
    args = parser.parse_args()

    # Initialize MCP3008
    mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

    # Initialize IMU
    try:
        bno = setup_imu()
    except Exception as e:
        print(f"IMU Initialization Error: {e}")
        sys.exit(1)

    # Calibrate IMU
    try:
        calibration_offsets = calibrate_imu(bno)
    except Exception as e:
        print(f"IMU Calibration Error: {e}")
        sys.exit(1)

    # Initialize MotorControlSystem
    motor_control_system = MotorControlSystem(mcp, mode=args.mode)
    motor_control_system.set_mode(args.mode)

    # Graceful shutdown handler
    def cleanup(signum, frame):
        print("\nShutting down gracefully...")
        motor_control_system.stop()
        sys.exit(0)

    # Register the cleanup function for SIGINT (Ctrl+C)
    signal.signal(signal.SIGINT, cleanup)

    print(f"\nStarting {args.mode} gait for 15 seconds...")

    # Open log file
    log_filename = f"imu_log_{args.mode}_{int(time.time())}.csv"
    with open(log_filename, 'w') as logfile:
        logfile.write("Timestamp,Roll,Pitch,Yaw\n")

        start_time = time.time()
        while time.time() - start_time < 15:
            roll, pitch, yaw = get_current_euler(bno, calibration_offsets)
            if None not in (roll, pitch, yaw):
                timestamp = time.time()
                logfile.write(f"{timestamp},{roll:.2f},{pitch:.2f},{yaw:.2f}\n")
            else:
                print("\nFailed to retrieve calibrated IMU data.")
            time.sleep(0.05)  # Adjust sampling rate as needed

    print(f"\nCompleted {args.mode} gait. IMU data saved to {log_filename}")

    # Stop the motor control system
    motor_control_system.stop()

if __name__ == "__main__":
    main()
