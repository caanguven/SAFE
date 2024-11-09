import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

# Constants
SPI_PORT = 0
SPI_DEVICE = 0
MOTOR_IN1 = 7
MOTOR_IN2 = 26
MOTOR_SPD = 18
POTENTIOMETER_CHANNEL = 0

# ADC range and dead zone threshold
ADC_MAX = 1023
DEAD_ZONE_THRESHOLD = 50  # Value to detect entry/exit from dead zone

# GPIO and PWM setup for motor control
GPIO.setmode(GPIO.BOARD)
GPIO.setup(MOTOR_IN1, GPIO.OUT)
GPIO.setup(MOTOR_IN2, GPIO.OUT)
GPIO.setup(MOTOR_SPD, GPIO.OUT)

# Set up PWM for motor speed control
motor_pwm = GPIO.PWM(MOTOR_SPD, 1000)  # 1000 Hz frequency
motor_pwm.start(50)  # Start with 50% speed

# Set up MCP3008
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

class MotorController:
    def __init__(self, adc_channel, target_position, tolerance=5):
        self.adc_channel = adc_channel
        self.target_position = target_position
        self.tolerance = tolerance  # Degrees of error allowed at the target
        self.position = 0  # Initial position
        self.direction = 'forward'
        self.in_dead_zone = False
        self.last_valid_position = None

    def read_adc_position(self):
        # Read ADC and convert to position within 330 degrees
        adc_value = mcp.read_adc(self.adc_channel)
        degrees = (adc_value / ADC_MAX) * 330.0
        return degrees, adc_value

    def set_motor_direction(self, direction):
        self.direction = direction
        if direction == 'forward':
            GPIO.output(MOTOR_IN1, GPIO.HIGH)
            GPIO.output(MOTOR_IN2, GPIO.LOW)
        elif direction == 'backward':
            GPIO.output(MOTOR_IN1, GPIO.LOW)
            GPIO.output(MOTOR_IN2, GPIO.HIGH)
        print(f"Motor set to rotate {direction}...")

    def update_position(self):
        # Read ADC position
        degrees, adc_value = self.read_adc_position()

        # Detect if we're in the dead zone
        if adc_value < DEAD_ZONE_THRESHOLD or adc_value > (ADC_MAX - DEAD_ZONE_THRESHOLD):
            print("[MotorController] In dead zone, estimating position...")
            self.in_dead_zone = True
            # Estimate the position based on direction and previous valid position
            if self.direction == 'forward':
                self.position = (self.position + 10) % 330  # Increment by an estimated step size
            else:
                self.position = (self.position - 10) % 330
        else:
            # We are not in the dead zone, update the actual position
            self.position = degrees
            self.in_dead_zone = False
            self.last_valid_position = self.position
            print(f"[MotorController] Updated Position: {self.position:.2f}°")

        return self.position

    def move_to_target(self):
        while True:
            current_position = self.update_position()
            error = self.target_position - current_position

            # Normalize error for circular movement (-165 to 165 range)
            if error > 165:
                error -= 330
            elif error < -165:
                error += 330

            print(f"[MotorController] Target: {self.target_position}°, Current: {current_position:.2f}°, Error: {error:.2f}°")

            # If within tolerance, stop the motor
            if abs(error) <= self.tolerance:
                self.stop_motor()
                print(f"[MotorController] Reached target position: {self.target_position}°")
                break

            # Adjust motor direction based on error
            if error > 0:
                self.set_motor_direction('forward')
            else:
                self.set_motor_direction('backward')

            # Set speed proportional to error, with a minimum speed threshold
            speed = min(100, max(30, abs(error)))
            motor_pwm.ChangeDutyCycle(speed)
            print(f"[MotorController] Setting speed: {speed}%")

            # Short delay for control loop frequency
            time.sleep(0.1)

    def stop_motor(self):
        GPIO.output(MOTOR_IN1, GPIO.LOW)
        GPIO.output(MOTOR_IN2, GPIO.LOW)
        motor_pwm.ChangeDutyCycle(0)
        print("[MotorController] Motor stopped.")

try:
    # Initialize MotorController with target position and tolerance
    controller = MotorController(adc_channel=POTENTIOMETER_CHANNEL, target_position=90, tolerance=5)

    # Start moving towards the target position
    controller.move_to_target()

except KeyboardInterrupt:
    print("Program interrupted by user")

finally:
    # Stop motor and cleanup GPIO
    controller.stop_motor()
    motor_pwm.stop()
    GPIO.cleanup()
    print("GPIO cleaned up")
