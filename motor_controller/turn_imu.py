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

# Clear any existing GPIO setup first
GPIO.setwarnings(False)
GPIO.cleanup()

# Constants
TOLERANCE = 6.0  # Degrees of acceptable error

# Motor GPIO Pin Configuration (Using BOARD mode numbering)
MOTOR1_IN1, MOTOR1_IN2, MOTOR1_SPD = 7, 26, 18
MOTOR2_IN1, MOTOR2_IN2, MOTOR2_SPD = 29, 22, 31
MOTOR3_IN1, MOTOR3_IN2, MOTOR3_SPD = 11, 32, 33
MOTOR4_IN1, MOTOR4_IN2, MOTOR4_SPD = 12, 13, 35

def setup_gpio():
    """Setup GPIO with proper error handling."""
    try:
        GPIO.setwarnings(False)
        GPIO.cleanup()
        GPIO.setmode(GPIO.BOARD)
        
        # Setup all motor pins
        pins = [
            MOTOR1_IN1, MOTOR1_IN2, MOTOR1_SPD,
            MOTOR2_IN1, MOTOR2_IN2, MOTOR2_SPD,
            MOTOR3_IN1, MOTOR3_IN2, MOTOR3_SPD,
            MOTOR4_IN1, MOTOR4_IN2, MOTOR4_SPD
        ]
        
        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        # Initialize PWM
        pwm1 = GPIO.PWM(MOTOR1_SPD, 1000)
        pwm2 = GPIO.PWM(MOTOR2_SPD, 1000)
        pwm3 = GPIO.PWM(MOTOR3_SPD, 1000)
        pwm4 = GPIO.PWM(MOTOR4_SPD, 1000)
        
        pwm1.start(0)
        pwm2.start(0)
        pwm3.start(0)
        pwm4.start(0)
        
        return pwm1, pwm2, pwm3, pwm4
        
    except Exception as e:
        print(f"GPIO Setup Error: {str(e)}")
        GPIO.cleanup()
        sys.exit(1)

class IMUSensor:
    def __init__(self):
        self.bno = None
        self.initialized = False
        self.last_yaw = 0
        self.calibration_offset = 0
        
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
                
                # Calibrate by taking initial reading
                print("Calibrating IMU...")
                self.calibration_offset = self._read_raw_yaw()
                print(f"Calibration offset: {self.calibration_offset:.1f}°")
                
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
    
    def _read_raw_yaw(self):
        """Read raw yaw value from IMU."""
        try:
            quat = self.bno.quaternion
            if quat is None:
                return self.last_yaw
                
            quat_i, quat_j, quat_k, quat_real = quat
            yaw = math.degrees(math.atan2(2 * (quat_real * quat_k + quat_i * quat_j),
                                        1 - 2 * (quat_j * quat_j + quat_k * quat_k)))
            return yaw
        except Exception as e:
            print(f"Error reading raw yaw: {str(e)}")
            return self.last_yaw
    
    def get_yaw(self):
        """Get calibrated yaw angle from IMU."""
        if not self.initialized:
            return self.last_yaw
            
        try:
            raw_yaw = self._read_raw_yaw()
            # Apply calibration offset and normalize to 0-360
            calibrated_yaw = (raw_yaw - self.calibration_offset) % 360
            self.last_yaw = calibrated_yaw
            return calibrated_yaw
            
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

class TurnController:
    def __init__(self, motors, tolerance=6.0):
        self.motors = motors
        self.tolerance = tolerance
        self.base_speed = 40
        self.min_speed = 30
        self.max_speed = 70
        self.last_error = 0
        self.integral = 0
        
    def calculate_speed(self, error):
        """Calculate motor speed using PID control."""
        Kp = 1.5  # Proportional gain
        Ki = 0.01  # Integral gain
        Kd = 0.5  # Derivative gain
        
        # Calculate PID terms
        self.integral += error
        derivative = error - self.last_error
        self.last_error = error
        
        # Calculate control signal
        control = (Kp * error) + (Ki * self.integral) + (Kd * derivative)
        
        # Calculate final speed
        speed = self.base_speed + control
        
        # Constrain speed between min and max values
        return min(max(abs(speed), self.min_speed), self.max_speed)
    
    def turn(self, current_yaw, target_yaw):
        """Execute turn based on current and target yaw angles."""
        # Calculate error (shortest path)
        error = target_yaw - current_yaw
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
            
        print(f"Current Yaw: {current_yaw:6.1f}°  Target: {target_yaw:6.1f}°  Error: {error:6.1f}°")
        
        # Check if within tolerance
        if abs(error) <= self.tolerance:
            print("Target reached!")
            self.stop_all_motors()
            return True
            
        # Calculate turn speed
        speed = self.calculate_speed(error)
        
        # Execute turn
        if error > 0:  # Turn right
            print(f"Turning right at {speed:.1f}% power")
            self.turn_right(speed)
        else:  # Turn left
            print(f"Turning left at {speed:.1f}% power")
            self.turn_left(speed)
            
        return False
    
    def turn_right(self, speed):
        self.motors['M1'].set_direction('forward')
        self.motors['M2'].set_direction('backward')
        self.motors['M3'].set_direction('forward')
        self.motors['M4'].set_direction('backward')
        for motor in self.motors.values():
            motor.set_speed(speed)
    
    def turn_left(self, speed):
        self.motors['M1'].set_direction('backward')
        self.motors['M2'].set_direction('forward')
        self.motors['M3'].set_direction('backward')
        self.motors['M4'].set_direction('forward')
        for motor in self.motors.values():
            motor.set_speed(speed)
    
    def stop_all_motors(self):
        for motor in self.motors.values():
            motor.stop()

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 turn_imu.py <target_angle>")
        print("Example: python3 turn_imu.py 30")
        sys.exit(1)
    
    try:
        target_angle = float(sys.argv[1])
        if target_angle < 0 or target_angle >= 360:
            print("Error: Target angle must be between 0 and 360 degrees")
            sys.exit(1)
    except ValueError:
        print("Error: Target angle must be a number")
        sys.exit(1)

    try:
        # Setup GPIO and get PWM objects
        pwm1, pwm2, pwm3, pwm4 = setup_gpio()
        
        # Initialize IMU
        print("\nInitializing IMU...")
        imu_sensor = IMUSensor()
        if not imu_sensor.initialize():
            print("Failed to initialize IMU. Exiting.")
            GPIO.cleanup()
            sys.exit(1)
        
        # Initialize motors
        print("\nInitializing motors...")
        motors = {
            'M1': MotorController("M1", MOTOR1_IN1, MOTOR1_IN2, pwm1),
            'M2': MotorController("M2", MOTOR2_IN1, MOTOR2_IN2, pwm2),
            'M3': MotorController("M3", MOTOR3_IN1, MOTOR3_IN2, pwm3),
            'M4': MotorController("M4", MOTOR4_IN1, MOTOR4_IN2, pwm4)
        }
        
        # Initialize turn controller
        turn_controller = TurnController(motors)
        
        print(f"\nStarting turn to {target_angle}°")
        print("Press Ctrl+C to stop")
        
        # Keep turning until target is reached
        while not turn_controller.turn(imu_sensor.get_yaw(), target_angle):
            time.sleep(0.1)
        
        print("\nTurn complete!")
            
    except KeyboardInterrupt:
        print("\nTurn interrupted by user")
    except Exception as e:
        print(f"\nError during operation: {str(e)}")
    finally:
        # Cleanup
        print("\nCleaning up...")
        try:
            if 'turn_controller' in locals():
                turn_controller.stop_all_motors()
        except:
            pass
        GPIO.cleanup()

if __name__ == "__main__":
    main()