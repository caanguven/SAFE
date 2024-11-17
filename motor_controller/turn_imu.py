import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import sys
import busio
import board
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
import math

# Constants for SPI and ADC
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023
TOLERANCE = 6.0  # Degrees of acceptable error

# Motor GPIO Pin Configuration
MOTOR1_IN1, MOTOR1_IN2, MOTOR1_SPD = 7, 26, 18
MOTOR2_IN1, MOTOR2_IN2, MOTOR2_SPD = 29, 22, 31
MOTOR3_IN1, MOTOR3_IN2, MOTOR3_SPD = 11, 32, 33
MOTOR4_IN1, MOTOR4_IN2, MOTOR4_SPD = 12, 13, 35

# GPIO setup at module level
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

# Set up PWM at module level
motor1_pwm = GPIO.PWM(MOTOR1_SPD, 1000)
motor1_pwm.start(0)
motor2_pwm = GPIO.PWM(MOTOR2_SPD, 1000)
motor2_pwm.start(0)
motor3_pwm = GPIO.PWM(MOTOR3_SPD, 1000)
motor3_pwm.start(0)
motor4_pwm = GPIO.PWM(MOTOR4_SPD, 1000)
motor4_pwm.start(0)

class IMUSensor:
    def __init__(self):
        self.bno = None
        self.initialized = False
        self.last_yaw = 0
        
    def initialize(self):
        """Initialize the IMU with error handling and retries."""
        MAX_RETRIES = 3
        retry_count = 0
        
        while retry_count < MAX_RETRIES:
            try:
                print("Initializing IMU...")
                i2c = busio.I2C(board.SCL, board.SDA)
                time.sleep(0.1)
                
                self.bno = BNO08X_I2C(i2c)
                time.sleep(0.5)
                
                print("Enabling rotation vector...")
                self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
                time.sleep(0.1)
                
                print("Testing IMU reading...")
                self.get_yaw()
                
                self.initialized = True
                print("IMU initialization successful!")
                return True
                
            except Exception as e:
                retry_count += 1
                print(f"IMU initialization attempt {retry_count} failed: {str(e)}")
                time.sleep(1)
                
                if retry_count >= MAX_RETRIES:
                    print("Failed to initialize IMU after maximum retries")
                    return False
    
    def get_yaw(self):
        """Get current yaw angle from IMU with error handling."""
        if not self.initialized:
            return self.last_yaw
            
        try:
            quat = self.bno.quaternion
            if quat is None:
                return self.last_yaw
                
            quat_i, quat_j, quat_k, quat_real = quat
            yaw = math.degrees(math.atan2(2 * (quat_real * quat_k + quat_i * quat_j),
                                        1 - 2 * (quat_j * quat_j + quat_k * quat_k)))
            yaw = (yaw + 360) % 360
            self.last_yaw = yaw
            return yaw
            
        except Exception as e:
            print(f"Error reading IMU: {str(e)}")
            return self.last_yaw

class MotorController:
    def __init__(self, name, in1, in2, pwm):
        self.name = name
        self.in1 = in1
        self.in2 = in2
        self.pwm = pwm

    def set_direction(self, direction):
        if direction == 'forward':
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        elif direction == 'backward':
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.LOW)

    def set_speed(self, speed):
        self.pwm.ChangeDutyCycle(abs(speed))

    def stop(self):
        self.set_direction('stop')
        self.pwm.ChangeDutyCycle(0)

def turn_to_angle(motors, imu_sensor, target_angle):
    """Turn the robot to reach the target angle based on IMU readings."""
    current_yaw = imu_sensor.get_yaw()
    
    # Calculate error (shortest path to target)
    error = target_angle - current_yaw
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    
    print(f"Current Yaw: {current_yaw:6.1f}°  Target: {target_angle:6.1f}°  Error: {error:6.1f}°")
    
    # Check if we're within tolerance
    if abs(error) <= TOLERANCE:
        print("Target reached!")
        return True
    
    # Calculate motor speed based on error magnitude
    base_speed = 40
    speed = min(max(abs(error) * 1.5, base_speed), 70)
    
    # Set motor directions based on turn direction
    if error > 0:  # Turn right
        print(f"Turning right at {speed:.1f}% power")
        motors['M1'].set_direction('forward')
        motors['M2'].set_direction('backward')
        motors['M3'].set_direction('forward')
        motors['M4'].set_direction('backward')
    else:  # Turn left
        print(f"Turning left at {speed:.1f}% power")
        motors['M1'].set_direction('backward')
        motors['M2'].set_direction('forward')
        motors['M3'].set_direction('backward')
        motors['M4'].set_direction('forward')
    
    # Set speed for all motors
    for motor in motors.values():
        motor.set_speed(speed)
    
    return False

def main():
    # Check command line arguments
    if len(sys.argv) != 2:
        print("Usage: python3 turn.py <target_angle>")
        print("Example: python3 turn.py 30")
        sys.exit(1)
    
    try:
        target_angle = float(sys.argv[1])
    except ValueError:
        print("Error: Target angle must be a number")
        sys.exit(1)
    
    # Initialize IMU
    imu_sensor = IMUSensor()
    if not imu_sensor.initialize():
        print("Failed to initialize IMU. Exiting.")
        GPIO.cleanup()
        sys.exit(1)
    
    # Initialize motors using the already set up PWM objects
    motors = {
        'M1': MotorController("M1", MOTOR1_IN1, MOTOR1_IN2, motor1_pwm),
        'M2': MotorController("M2", MOTOR2_IN1, MOTOR2_IN2, motor2_pwm),
        'M3': MotorController("M3", MOTOR3_IN1, MOTOR3_IN2, motor3_pwm),
        'M4': MotorController("M4", MOTOR4_IN1, MOTOR4_IN2, motor4_pwm)
    }
    
    print(f"Starting turn to {target_angle}°")
    print("Press Ctrl+C to stop")
    
    try:
        # Keep turning until target is reached
        while not turn_to_angle(motors, imu_sensor, target_angle):
            time.sleep(0.1)
        
        # Stop all motors when target is reached
        for motor in motors.values():
            motor.stop()
            
    except KeyboardInterrupt:
        print("\nTurn interrupted by user")
    except Exception as e:
        print(f"Error during operation: {str(e)}")
    finally:
        # Cleanup
        print("Cleaning up...")
        try:
            for motor in motors.values():
                motor.stop()
        except:
            pass
        GPIO.cleanup()

if __name__ == "__main__":
    main()