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
                time.sleep(0.1)  # Short delay before initialization
                
                self.bno = BNO08X_I2C(i2c)
                time.sleep(0.5)  # Give the sensor time to settle
                
                # Enable the rotation vector feature
                print("Enabling rotation vector...")
                self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
                time.sleep(0.1)  # Wait for feature to be enabled
                
                # Test reading
                print("Testing IMU reading...")
                self.get_yaw()
                
                self.initialized = True
                print("IMU initialization successful!")
                return True
                
            except Exception as e:
                retry_count += 1
                print(f"IMU initialization attempt {retry_count} failed: {str(e)}")
                time.sleep(1)  # Wait before retrying
                
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
            
            # Convert quaternion to Euler angles
            yaw = math.degrees(math.atan2(2 * (quat_real * quat_k + quat_i * quat_j),
                                        1 - 2 * (quat_j * quat_j + quat_k * quat_k)))
            
            # Normalize yaw to 0-360 range
            yaw = (yaw + 360) % 360
            self.last_yaw = yaw
            return yaw
            
        except Exception as e:
            print(f"Error reading IMU: {str(e)}")
            return self.last_yaw

class MotorController:
    def __init__(self, name, in1, in2, spd):
        self.name = name
        self.in1 = in1
        self.in2 = in2
        self.spd = spd
        
        # Setup GPIO
        GPIO.setup(in1, GPIO.OUT)
        GPIO.setup(in2, GPIO.OUT)
        GPIO.setup(spd, GPIO.OUT)
        
        # Setup PWM
        self.pwm = GPIO.PWM(spd, 1000)
        self.pwm.start(0)

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
    speed = min(max(abs(error) * 1.5, base_speed), 70)  # Scale speed with error
    
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
    
    # Initialize GPIO
    GPIO.setmode(GPIO.BOARD)
    
    # Initialize IMU
    imu_sensor = IMUSensor()
    if not imu_sensor.initialize():
        print("Failed to initialize IMU. Exiting.")
        sys.exit(1)
    
    # Initialize motors
    motors = {
        'M1': MotorController("M1", MOTOR1_IN1, MOTOR1_IN2, MOTOR1_SPD),
        'M2': MotorController("M2", MOTOR2_IN1, MOTOR2_IN2, MOTOR2_SPD),
        'M3': MotorController("M3", MOTOR3_IN1, MOTOR3_IN2, MOTOR3_SPD),
        'M4': MotorController("M4", MOTOR4_IN1, MOTOR4_IN2, MOTOR4_SPD)
    }
    
    print(f"Starting turn to {target_angle}°")
    print("Press Ctrl+C to stop")
    
    try:
        # Keep turning until target is reached
        while not turn_to_angle(motors, imu_sensor, target_angle):
            time.sleep(0.1)  # Small delay between adjustments
        
        # Stop all motors when target is reached
        for motor in motors.values():
            motor.stop()
            
    except KeyboardInterrupt:
        print("\nTurn interrupted by user")
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        # Cleanup
        print("Cleaning up...")
        for motor in motors.values():
            motor.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()