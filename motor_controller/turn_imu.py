import warnings
import argparse
warnings.filterwarnings("ignore", category=RuntimeWarning, module="adafruit_blinka.microcontroller.generic_linux.i2c")
warnings.filterwarnings("ignore", category=RuntimeWarning)  # Added to suppress more warnings

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

# Configure basic logging
logging.basicConfig(
    level=logging.INFO,
    format='%(message)s',
    handlers=[logging.StreamHandler()]
)

# Suppress GPIO warnings
GPIO.setwarnings(False)

# GPIO Pins configuration remains the same
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

# Motor setup remains the same
def setup_motors():
    GPIO.setmode(GPIO.BCM)
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
        pwm = GPIO.PWM(spd, 100)
        pwm.start(0)
        motor_pwms[f"M{i}"] = pwm
    
    return motor_pins, motor_pwms

def setup_imu():
    """Initialize IMU with retry mechanism"""
    max_retries = 3
    retry_delay = 2  # seconds
    
    for attempt in range(max_retries):
        try:
            print(f"Attempting to initialize IMU (attempt {attempt + 1}/{max_retries})...")
            
            # Create I2C interface
            i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)  # Reduced frequency
            
            # Wait for I2C bus to be ready
            time.sleep(0.5)
            
            # Initialize BNO08x
            bno = BNO08X_I2C(i2c)
            
            # Wait for sensor to be ready
            time.sleep(1)
            
            # Enable rotation vector feature
            bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
            
            # Wait for feature to be enabled
            time.sleep(0.5)
            
            # Verify sensor is working
            quat = bno.quaternion
            if quat is None:
                raise RuntimeError("Unable to read quaternion data")
                
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
    max_samples = 10
    timeout = 30  # seconds
    start_time = time.time()
    
    while len(samples) < max_samples and time.time() - start_time < timeout:
        try:
            print(f"Collecting samples... {len(samples) + 1}/{max_samples}", end='\r')
            quat = bno.quaternion
            
            if quat is not None and len(quat) == 4:
                _, _, yaw = quaternion_to_euler(*quat)
                if not math.isnan(yaw):
                    samples.append(yaw)
                    time.sleep(0.1)
            else:
                print("\nInvalid quaternion reading, retrying...")
                time.sleep(0.5)
                
        except Exception as e:
            print(f"\nError during calibration: {str(e)}")
            time.sleep(0.5)
    
    if len(samples) >= max_samples / 2:  # Accept if we have at least half the samples
        calibration_offset = sum(samples) / len(samples)
        print(f"\nCalibration complete. Reference angle: {calibration_offset:.2f}°")
        return calibration_offset
    else:
        raise RuntimeError("Failed to collect enough valid samples for calibration")

def get_current_yaw(bno, calibration_offset, retries=3):
    """Get current yaw with retry mechanism"""
    for _ in range(retries):
        try:
            quat = bno.quaternion
            if quat is not None and len(quat) == 4 and not any(math.isnan(x) for x in quat):
                _, _, yaw = quaternion_to_euler(*quat)
                return (yaw - calibration_offset) % 360
        except Exception:
            time.sleep(0.1)
    return None

# Motor control functions remain the same
def set_motor_direction(motor_pins, motor, direction):
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

def set_motor_speed(motor_pwms, motor, speed):
    motor_pwms[motor].ChangeDutyCycle(speed)

def stop_all_motors(motor_pins, motor_pwms):
    for i, _ in enumerate(motor_pins, 1):
        motor = f"M{i}"
        set_motor_direction(motor_pins, motor, 'stop')
        set_motor_speed(motor_pwms, motor, 0)

def main():
    parser = argparse.ArgumentParser(description='Turn robot based on IMU readings')
    parser.add_argument('angle', type=float, help='Angle to turn (degrees, positive for right, negative for left)')
    args = parser.parse_args()
    
    if abs(args.angle) > 360:
        print("Error: Angle must be between -360 and 360 degrees")
        sys.exit(1)
    
    motor_pins, motor_pwms = setup_motors()
    
    def cleanup():
        stop_all_motors(motor_pins, motor_pwms)
        for pwm in motor_pwms.values():
            pwm.stop()
        GPIO.cleanup()
    
    try:
        # Initialize IMU
        bno = setup_imu()
        
        # Calibrate IMU
        calibration_offset = calibrate_imu(bno)
        
        # Get initial yaw
        initial_yaw = get_current_yaw(bno, calibration_offset)
        if initial_yaw is None:
            raise RuntimeError("Failed to get initial yaw reading")
        
        # Set up turn parameters
        turn_direction = 'right' if args.angle > 0 else 'left'
        target_angle = abs(args.angle)
        motor_speed = 70
        
        print(f"\nStarting {turn_direction} turn of {target_angle}° from {initial_yaw:.2f}°")
        
        # Configure motors
        if turn_direction == 'right':
            set_motor_direction(motor_pins, 'M1', 'forward')
            set_motor_direction(motor_pins, 'M2', 'backward')
            set_motor_direction(motor_pins, 'M3', 'forward')
            set_motor_direction(motor_pins, 'M4', 'backward')
        else:
            set_motor_direction(motor_pins, 'M1', 'backward')
            set_motor_direction(motor_pins, 'M2', 'forward')
            set_motor_direction(motor_pins, 'M3', 'backward')
            set_motor_direction(motor_pins, 'M4', 'forward')
        
        # Set speeds
        for motor in motor_pwms:
            set_motor_speed(motor_pwms, motor, motor_speed)
        
        # Main turn loop
        while True:
            current_yaw = get_current_yaw(bno, calibration_offset)
            if current_yaw is None:
                raise RuntimeError("Lost IMU connection")
            
            # Calculate angle turned
            if turn_direction == 'right':
                angle_turned = (current_yaw - initial_yaw) % 360
                if angle_turned > 180:
                    angle_turned -= 360
            else:
                angle_turned = (initial_yaw - current_yaw) % 360
                if angle_turned > 180:
                    angle_turned -= 360
            
            print(f"Current angle: {current_yaw:.2f}°, Turned: {abs(angle_turned):.2f}°    ", end='\r')
            
            if abs(angle_turned) >= target_angle:
                print(f"\nTarget angle reached! Final angle: {current_yaw:.2f}°")
                break
            
            time.sleep(0.05)
        
    except KeyboardInterrupt:
        print("\nOperation cancelled by user")
    except Exception as e:
        print(f"\nError: {str(e)}")
    finally:
        cleanup()

if __name__ == "__main__":
    main()