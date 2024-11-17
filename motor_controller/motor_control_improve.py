import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import curses
import math

# Constants for SPI and ADC
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023
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

# Set up PWM
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

class MotorController:
    def __init__(self, name, in1, in2, pwm, adc_channel, encoder_flipped=False):
        self.name = name
        self.in1 = in1
        self.in2 = in2
        self.pwm = pwm
        self.adc_channel = adc_channel
        self.encoder_flipped = encoder_flipped
        self.last_valid_position = None

    def read_position(self):
        raw_value = mcp.read_adc(self.adc_channel)
        
        if self.encoder_flipped:
            raw_value = ADC_MAX - raw_value
            
        degrees = (raw_value / ADC_MAX) * 330.0
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

    def set_speed(self, speed):
        self.pwm.ChangeDutyCycle(abs(speed))

    def stop_motor(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)

class TurnController:
    def __init__(self, motors, tolerance=6.0):
        self.motors = motors
        self.tolerance = tolerance
        self.base_speed = 40  # Base speed for turning
        self.min_speed = 30   # Minimum speed to maintain movement
        self.max_speed = 70   # Maximum speed for faster turning

    def calculate_turn_speed(self, error):
        """Calculate motor speed based on how far we are from target."""
        # Proportional control
        speed = abs(error) * 2  # Multiply error by 2 for more aggressive response
        
        # Constrain speed between min and max values
        speed = max(self.min_speed, min(speed, self.max_speed))
        return speed

    def turn_to_angle(self, current_yaw, target_yaw):
        """
        Implement turning logic based on yaw difference.
        Returns True if within tolerance, False otherwise.
        """
        # Calculate the shortest turning direction
        error = target_yaw - current_yaw
        
        # Normalize error to [-180, 180]
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        print(f"Current Yaw: {current_yaw:.1f}°, Target: {target_yaw:.1f}°, Error: {error:.1f}°")

        # Check if we're within tolerance
        if abs(error) <= self.tolerance:
            print("Within tolerance - stopping")
            self.stop_all_motors()
            return True

        # Determine turn direction and set motor speeds
        turn_speed = self.calculate_turn_speed(error)
        
        if error > 0:  # Turn right
            print(f"Turning right at speed {turn_speed:.1f}%")
            self.turn_right(turn_speed)
        else:  # Turn left
            print(f"Turning left at speed {turn_speed:.1f}%")
            self.turn_left(turn_speed)

        return False

    def turn_right(self, speed):
        # For right turn: Left motors forward, Right motors backward
        self.motors['M1'].set_motor_direction('forward')
        self.motors['M3'].set_motor_direction('forward')
        self.motors['M2'].set_motor_direction('backward')
        self.motors['M4'].set_motor_direction('backward')
        
        for motor in self.motors.values():
            motor.set_speed(speed)

    def turn_left(self, speed):
        # For left turn: Left motors backward, Right motors forward
        self.motors['M1'].set_motor_direction('backward')
        self.motors['M3'].set_motor_direction('backward')
        self.motors['M2'].set_motor_direction('forward')
        self.motors['M4'].set_motor_direction('forward')
        
        for motor in self.motors.values():
            motor.set_speed(speed)

    def stop_all_motors(self):
        for motor in self.motors.values():
            motor.stop_motor()

def main(stdscr):
    # Initialize motors
    motors = {
        'M1': MotorController("M1", MOTOR1_IN1, MOTOR1_IN2, motor1_pwm, MOTOR1_ADC_CHANNEL),
        'M2': MotorController("M2", MOTOR2_IN1, MOTOR2_IN2, motor2_pwm, MOTOR2_ADC_CHANNEL, True),
        'M3': MotorController("M3", MOTOR3_IN1, MOTOR3_IN2, motor3_pwm, MOTOR3_ADC_CHANNEL),
        'M4': MotorController("M4", MOTOR4_IN1, MOTOR4_IN2, motor4_pwm, MOTOR4_ADC_CHANNEL, True)
    }

    turn_controller = TurnController(motors)
    
    # Initialize curses for display
    curses.cbreak()
    curses.noecho()
    stdscr.nodelay(True)
    stdscr.keypad(True)

    # Display instructions
    stdscr.addstr(0, 0, "IMU-based Turn Control System")
    stdscr.addstr(2, 0, "Press 'q' to quit")
    stdscr.refresh()

    try:
        target_yaw = 0  # Initial target yaw
        last_display_time = time.time()

        while True:
            # Check for quit command
            key = stdscr.getch()
            if key == ord('q'):
                break

            # Get current yaw from IMU data (replace with your IMU reading)
            current_yaw = imu_data.get('yaw', 0)  # You'll need to implement this
            
            # Update turn control
            turn_controller.turn_to_angle(current_yaw, target_yaw)

            # Update display every 100ms
            current_time = time.time()
            if current_time - last_display_time >= 0.1:
                stdscr.addstr(4, 0, f"Current Yaw: {current_yaw:6.1f}°")
                stdscr.addstr(5, 0, f"Target Yaw:  {target_yaw:6.1f}°")
                stdscr.addstr(6, 0, f"Error:       {(target_yaw - current_yaw):6.1f}°")
                stdscr.refresh()
                last_display_time = current_time

            time.sleep(0.02)  # Small delay for system stability

    except Exception as e:
        stdscr.addstr(8, 0, f"Error: {str(e)}")
        stdscr.refresh()
        time.sleep(2)
    finally:
        # Cleanup
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

if __name__ == "__main__":
    curses.wrapper(main)