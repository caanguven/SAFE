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
SAWTOOTH_PERIOD = 5
SAWTOOTH_STEP = 5  # Step size for sawtooth wave increments

# GPIO Pins for Motor 1
MOTOR1_IN1 = 7
MOTOR1_IN2 = 26
MOTOR1_SPD = 18
MOTOR1_ADC_CHANNEL = 0

# GPIO Pins for Motor 3
MOTOR3_IN1 = 11
MOTOR3_IN2 = 32
MOTOR3_SPD = 33
MOTOR3_ADC_CHANNEL = 2

# GPIO setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(MOTOR1_IN1, GPIO.OUT)
GPIO.setup(MOTOR1_IN2, GPIO.OUT)
GPIO.setup(MOTOR1_SPD, GPIO.OUT)
GPIO.setup(MOTOR3_IN1, GPIO.OUT)
GPIO.setup(MOTOR3_IN2, GPIO.OUT)
GPIO.setup(MOTOR3_SPD, GPIO.OUT)

# Set up PWM for motor speed control
motor1_pwm = GPIO.PWM(MOTOR1_SPD, 1000)
motor1_pwm.start(0)
motor3_pwm = GPIO.PWM(MOTOR3_SPD, 1000)
motor3_pwm.start(0)

# Set up MCP3008
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

class MotorController:
    def __init__(self, name, in1, in2, pwm, adc_channel, target_position):
        self.name = name
        self.in1 = in1
        self.in2 = in2
        self.pwm = pwm
        self.adc_channel = adc_channel
        self.target_position = target_position
        self.position = 0  # Initial position
        self.in_dead_zone = False

    def read_adc_position(self):
        adc_value = mcp.read_adc(self.adc_channel)
        degrees = (adc_value / ADC_MAX) * 330.0
        return degrees, adc_value

    def set_motor_direction(self, direction):
        if direction == 'forward':
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        elif direction == 'backward':
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)

    def update_position(self):
        degrees, adc_value = self.read_adc_position()

        if adc_value < DEAD_ZONE_THRESHOLD or adc_value > (ADC_MAX - DEAD_ZONE_THRESHOLD):
            self.in_dead_zone = True
            print(f"[{self.name}] In dead zone, holding last position...")
            return self.position
        else:
            self.position = degrees
            self.in_dead_zone = False
            print(f"[{self.name}] Updated Position: {self.position:.2f}°")
            return self.position

    def move_toward_target(self):
        error = self.target_position - self.position
        if error > 165:
            error -= 330
        elif error < -165:
            error += 330

        if abs(error) <= SAWTOOTH_STEP:
            self.pwm.ChangeDutyCycle(0)
            return True
        else:
            direction = 'forward' if error > 0 else 'backward'
            self.set_motor_direction(direction)
            self.pwm.ChangeDutyCycle(min(100, max(30, abs(error))))
            return False

def synchronized_sawtooth(motor1, motor3, duration):
    initial_offset = motor3.target_position - motor1.target_position
    start_time = time.time()
    
    while True:
        elapsed_time = time.time() - start_time
        wave_position = (elapsed_time % duration) * (330.0 / duration)

        motor1.target_position = wave_position % 330
        motor3.target_position = (wave_position + initial_offset) % 330

        motor1_reached = motor1.move_toward_target()
        motor3_reached = motor3.move_toward_target()

        print(f"[{motor1.name}] Target: {motor1.target_position:.2f}°, Position: {motor1.position:.2f}°")
        print(f"[{motor3.name}] Target: {motor3.target_position:.2f}°, Position: {motor3.position:.2f}°")

        if motor1_reached and motor3_reached:
            break

        time.sleep(0.1)  # Small delay for loop control

# Parse command-line arguments for target positions
parser = argparse.ArgumentParser(description="Move motors to target positions.")
parser.add_argument("motor1_target", type=int, nargs="?", default=90, help="Target position for Motor 1 (default: 90)")
parser.add_argument("motor3_target", type=int, nargs="?", default=270, help="Target position for Motor 3 (default: 270)")
args = parser.parse_args()

try:
    motor1_controller = MotorController(
        name="Motor 1", in1=MOTOR1_IN1, in2=MOTOR1_IN2, pwm=motor1_pwm,
        adc_channel=MOTOR1_ADC_CHANNEL, target_position=args.motor1_target
    )
    motor3_controller = MotorController(
        name="Motor 3", in1=MOTOR3_IN1, in2=MOTOR3_IN2, pwm=motor3_pwm,
        adc_channel=MOTOR3_ADC_CHANNEL, target_position=args.motor3_target
    )

    # Move both motors to initial positions
    print(f"Moving to initial positions: Motor 1 to {args.motor1_target}°, Motor 3 to {args.motor3_target}°")
    
    motor1_reached = motor1_controller.move_toward_target()
    motor3_reached = motor3_controller.move_toward_target()
    while not (motor1_reached and motor3_reached):
        motor1_reached = motor1_controller.move_toward_target()
        motor3_reached = motor3_controller.move_toward_target()
        time.sleep(0.1)
    
    print("Calibration complete. Both motors reached initial positions.")

    # Begin synchronized sawtooth wave movement
    print("Starting synchronized sawtooth wave pattern for both motors")
    while True:
        synchronized_sawtooth(motor1_controller, motor3_controller, SAWTOOTH_PERIOD)

except KeyboardInterrupt:
    print("Program interrupted by user")

finally:
    motor1_controller.pwm.ChangeDutyCycle(0)
    motor3_controller.pwm.ChangeDutyCycle(0)
    GPIO.cleanup()
    print("GPIO cleaned up")
