import warnings
import argparse
warnings.filterwarnings("ignore", category=RuntimeWarning, module="adafruit_blinka.microcontroller.generic_linux.i2c")

import RPi.GPIO as GPIO
import time
import busio
import board
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C
import sys
import logging
import math
import signal

# Configure logging - only INFO and above
logging.basicConfig(
    level=logging.INFO,
    format='%(message)s',
    handlers=[
        logging.FileHandler("turn_based_on_yaw.log"),
        logging.StreamHandler()
    ]
)

# Suppress GPIO warnings
GPIO.setwarnings(False)

# GPIO Pins for Motor Control (Using BCM numbering)
MOTOR1_IN1 = 4    # Left Front
MOTOR1_IN2 = 7
MOTOR1_SPD = 24

MOTOR2_IN1 = 5    # Right Front
MOTOR2_IN2 = 25
MOTOR2_SPD = 6

MOTOR3_IN1 = 17   # Left Rear
MOTOR3_IN2 = 12
MOTOR3_SPD = 13

MOTOR4_IN1 = 18   # Right Rear
MOTOR4_IN2 = 27
MOTOR4_SPD = 19

def signal_handler(sig, frame):
    print("\nProgram interrupted. Cleaning up...")
    stop_all_motors()
    for pwm in motor_pwms.values():
        pwm.stop()
    GPIO.cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Setup GPIO pins
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

# Initialize PWM for all motors with 100Hz frequency
motor_pwms = {}
for i, (in1, in2, spd) in enumerate(motor_pins, start=1):
    pwm = GPIO.PWM(spd, 100)
    pwm.start(0)
    motor_pwms[f"M{i}"] = pwm

def quaternion_to_euler(quat_i, quat_j, quat_k, quat_real):
    roll = math.atan2(2 * (quat_real * quat_i + quat_j * quat_k),
                     1 - 2 * (quat_i**2 + quat_j**2))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (quat_real * quat_j - quat_k * quat_i))))
    yaw = math.atan2(2 * (quat_real * quat_k + quat_i * quat_j),
                    1 - 2 * (quat_j**2 + quat_k**2))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def get_shortest_angle(angle1, angle2):
    diff = (angle2 - angle1) % 360
    if diff > 180:
        diff -= 360
    return diff

def verify_imu_data(bno):
    try:
        quat = bno.quaternion
        if quat is None or any(math.isnan(x) for x in quat) or any(abs(x) > 1.0 for x in quat):
            return False
        return True
    except Exception:
        return False

def setup_imu():
    try:
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        bno = BNO08X_I2C(i2c)
        time.sleep(1)
        bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
        time.sleep(1)
        
        if not verify_imu_data(bno):
            raise RuntimeError("IMU initialization failed")
        
        print("IMU initialized successfully")
        return bno
    except Exception as e:
        print(f"Failed to initialize IMU: {e}")
        GPIO.cleanup()
        sys.exit(1)

def calibrate_imu(bno):
    print("Calibrating IMU (stay still)...")
    samples = []
    for i in range(10):
        print(f"Calibrating... {i+1}/10", end='\r')
        quat = bno.quaternion
        if quat is not None:
            _, _, yaw = quaternion_to_euler(*quat)
            samples.append(yaw)
        time.sleep(0.1)
    
    if samples:
        calibration_offset = sum(samples) / len(samples)
        print(f"\nCalibration complete. Reference angle: {calibration_offset:.2f}°")
        return calibration_offset
    else:
        print("Calibration failed!")
        GPIO.cleanup()
        sys.exit(1)

def get_current_yaw(bno, calibration_offset):
    try:
        quat = bno.quaternion
        if quat is not None:
            _, _, yaw = quaternion_to_euler(*quat)
            return (yaw - calibration_offset) % 360
        return None
    except Exception:
        return None

def set_motor_direction(motor, direction):
    in1, in2, _ = motor_pins[int(motor[1])-1]
    if direction == 'forward':
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    elif direction == 'backward':
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
    else:  # stop
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)

def set_motor_speed(motor, speed):
    motor_pwms[motor].ChangeDutyCycle(speed)

def stop_all_motors():
    for motor in motor_pwms:
        set_motor_direction(motor, 'stop')
        set_motor_speed(motor, 0)
    print("\nMotors stopped")

def turn_robot(angle):
    # Initialize IMU
    bno = setup_imu()
    calibration_offset = calibrate_imu(bno)
    
    # Determine turn direction
    turn_direction = 'right' if angle > 0 else 'left'
    target_angle = abs(angle)
    motor_speed = 70
    
    # Get initial yaw
    initial_yaw = get_current_yaw(bno, calibration_offset)
    if initial_yaw is None:
        print("Failed to get initial yaw reading")
        return False
    
    print(f"\nStarting {turn_direction} turn of {target_angle}° from {initial_yaw:.2f}°")
    
    # Configure motors based on direction
    if turn_direction == 'right':
        set_motor_direction('M1', 'forward')   # Left Front
        set_motor_direction('M2', 'backward')  # Right Front
        set_motor_direction('M3', 'forward')   # Left Rear
        set_motor_direction('M4', 'backward')  # Right Rear
    else:  # left
        set_motor_direction('M1', 'backward')  # Left Front
        set_motor_direction('M2', 'forward')   # Right Front
        set_motor_direction('M3', 'backward')  # Left Rear
        set_motor_direction('M4', 'forward')   # Right Rear
    
    # Set speed for all motors
    for motor in motor_pwms:
        set_motor_speed(motor, motor_speed)
    
    try:
        while True:
            if not verify_imu_data(bno):
                print("IMU data verification failed")
                return False
            
            current_yaw = get_current_yaw(bno, calibration_offset)
            if current_yaw is None:
                print("Failed to get current yaw")
                return False
            
            # Calculate angle turned
            if turn_direction == 'right':
                angle_turned = get_shortest_angle(initial_yaw, current_yaw)
            else:
                angle_turned = get_shortest_angle(current_yaw, initial_yaw)
            
            print(f"Current angle: {current_yaw:.2f}°, Turned: {abs(angle_turned):.2f}°    ", end='\r')
            
            # Check if target angle reached
            if abs(angle_turned) >= target_angle:
                print(f"\nTarget angle reached! Final angle: {current_yaw:.2f}°")
                return True
            
            time.sleep(0.05)
            
    except Exception as e:
        print(f"\nError during turn: {e}")
        return False
    finally:
        stop_all_motors()

def main():
    parser = argparse.ArgumentParser(description='Turn robot based on IMU readings. Positive angle for right turn, negative for left turn.')
    parser.add_argument('angle', type=float, help='Angle to turn (degrees)')
    args = parser.parse_args()
    
    if abs(args.angle) > 360:
        print("Error: Angle must be between -360 and 360 degrees")
        sys.exit(1)
    
    try:
        if turn_robot(args.angle):
            print("Turn completed successfully")
        else:
            print("Turn failed")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        for pwm in motor_pwms.values():
            pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()