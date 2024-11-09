import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import argparse
import threading

# Constants for SPI and ADC
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023
DEAD_ZONE_THRESHOLD = 50
SAWTOOTH_PERIOD = 5000  # Period of sawtooth wave in milliseconds
SAWTOOTH_AMPLITUDE = 330  # Max angle (full rotation)

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

class SawtoothWaveGenerator:
    def __init__(self, period, amplitude, initial_angle):
        self.period = period
        self.amplitude = amplitude
        self.initial_angle = initial_angle
        self.start_time = self.current_millis()

    def current_millis(self):
        return int(time.time() * 1000)

    def get_set_position(self):
        t = self.current_millis() - self.start_time
        normalized_wave = (t % self.period) * (self.amplitude / self.period)
        return (normalized_wave + self.initial_angle) % 360

class MotorController:
    def __init__(self, name, in1, in2, pwm, adc_channel):
        self.name = name
        self.in1 = in1
        self.in2 = in2
        self.pwm = pwm
        self.adc_channel = adc_channel
        self.position = 0

    def read_adc_position(self):
        adc_value = mcp.read_adc(self.adc_channel)
        degrees = (adc_value / ADC_MAX) * 330.0
        self.position = degrees
        return degrees

    def set_motor_direction(self, direction):
        if direction == 'forward':
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        elif direction == 'backward':
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)

    def move_to_position(self, target_position):
        current_position = self.read_adc_position()
        error = target_position - current_position

        if error > 0:
            self.set_motor_direction('forward')
        else:
            self.set_motor_direction('backward')

        speed = min(100, max(30, abs(error) * 0.3))  # Scale speed with error
        self.pwm.ChangeDutyCycle(speed)

    def stop_motor(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)

    def follow_sawtooth_wave(self, wave_generator):
        while True:
            set_position = wave_generator.get_set_position()
            self.move_to_position(set_position)
            print(f"[{self.name}] Set Position: {set_position:.2f}°, Actual Position: {self.position:.2f}°")
            time.sleep(0.05)  # Short delay for control frequency

def control_motors(motor1_controller, motor3_controller, motor1_wave, motor3_wave):
    while True:
        # Retrieve sawtooth set positions for both motors
        motor1_set_position = motor1_wave.get_set_position()
        motor3_set_position = motor3_wave.get_set_position()

        # Move both motors to their respective positions
        motor1_controller.move_to_position(motor1_set_position)
        motor3_controller.move_to_position(motor3_set_position)

        # Print the set and actual positions for each motor
        print(f"[Motor 1] Set Position: {motor1_set_position:.2f}°, Actual Position: {motor1_controller.position:.2f}°")
        print(f"[Motor 3] Set Position: {motor3_set_position:.2f}°, Actual Position: {motor3_controller.position:.2f}°")
        
        time.sleep(0.05)  # Control frequency delay

# Parse command-line arguments for target positions
parser = argparse.ArgumentParser(description="Move motors to target positions.")
parser.add_argument("motor1_target", type=int, nargs="?", default=90, help="Target position for Motor 1 (default: 90)")
parser.add_argument("motor3_target", type=int, nargs="?", default=270, help="Target position for Motor 3 (default: 270)")
args = parser.parse_args()

motor1_target = args.motor1_target
motor3_target = args.motor3_target

try:
    motor1_controller = MotorController(
        name="Motor 1", in1=MOTOR1_IN1, in2=MOTOR1_IN2, pwm=motor1_pwm, adc_channel=MOTOR1_ADC_CHANNEL
    )
    motor3_controller = MotorController(
        name="Motor 3", in1=MOTOR3_IN1, in2=MOTOR3_IN2, pwm=motor3_pwm, adc_channel=MOTOR3_ADC_CHANNEL
    )

    # Calibrate motors to initial positions
    print(f"Calibrating: Motor 1 to {motor1_target}° and Motor 3 to {motor3_target}°")
    motor1_controller.move_to_position(motor1_target)
    motor3_controller.move_to_position(motor3_target)
    time.sleep(5)

    print("Starting synchronized sawtooth wave pattern")

    # Initialize sawtooth wave generators for each motor
    motor1_wave = SawtoothWaveGenerator(SAWTOOTH_PERIOD, SAWTOOTH_AMPLITUDE, motor1_target)
    motor3_wave = SawtoothWaveGenerator(SAWTOOTH_PERIOD, SAWTOOTH_AMPLITUDE, motor3_target)

    # Run both motors in sawtooth wave pattern in a single loop
    control_motors(motor1_controller, motor3_controller, motor1_wave, motor3_wave)

except KeyboardInterrupt:
    print("Program interrupted by user")

finally:
    motor1_controller.stop_motor()
    motor3_controller.stop_motor()
    motor1_pwm.stop()
    motor3_pwm.stop()
    GPIO.cleanup()
    print("GPIO cleaned up")
