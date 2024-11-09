import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import argparse
import math

# Constants for SPI and ADC
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023
DEAD_ZONE_THRESHOLD = 50
SAWTOOTH_PERIOD = 5

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

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()

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

class MotorController:
    def __init__(self, name, in1, in2, pwm, adc_channel, target_position, tolerance=5):
        self.name = name
        self.in1 = in1
        self.in2 = in2
        self.pwm = pwm
        self.adc_channel = adc_channel
        self.target_position = target_position
        self.tolerance = tolerance
        self.position = 0
        self.direction = 'forward'
        self.in_dead_zone = False
        self.last_valid_position = None
        self.dead_zone_counter = 0  # Counter to slow position updates in dead zone
        self.pid = PIDController(Kp=0.6, Ki=0.1, Kd=0.02)  # Adjusted PID coefficients

    def read_adc_position(self):
        adc_value = mcp.read_adc(self.adc_channel)
        degrees = (adc_value / ADC_MAX) * 330.0
        return degrees, adc_value

    def set_motor_direction(self, direction):
        self.direction = direction
        if direction == 'forward':
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        elif direction == 'backward':
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
        print(f"[{self.name}] Motor set to rotate {direction}...")

    def update_position(self):
        degrees, adc_value = self.read_adc_position()

        # Check for dead zone
        if adc_value < DEAD_ZONE_THRESHOLD or adc_value > (ADC_MAX - DEAD_ZONE_THRESHOLD):
            print(f"[{self.name}] In dead zone, estimating position...")

            # Use a delay counter to slow down dead zone updates
            if self.dead_zone_counter >= 2:
                if self.direction == 'forward':
                    self.position = (self.position + 5) % 330
                else:
                    self.position = (self.position - 5) % 330
                self.dead_zone_counter = 0
            else:
                self.dead_zone_counter += 1

            self.in_dead_zone = True
        else:
            # Valid position reading
            self.position = degrees
            self.in_dead_zone = False
            self.last_valid_position = self.position
            print(f"[{self.name}] Updated Position: {self.position:.2f}°")

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

            print(f"[{self.name}] Target: {self.target_position}°, Current: {current_position:.2f}°, Error: {error:.2f}°")

            # Stop if within tolerance
            if abs(error) <= self.tolerance:
                self.stop_motor()
                print(f"[{self.name}] Reached target position: {self.target_position}°")
                break

            control_signal = self.pid.compute(error)
            self.set_motor_direction('forward' if control_signal > 0 else 'backward')

            # Adjust speed proportional to the control signal with a minimum threshold
            speed = min(100, max(30, abs(control_signal)))
            self.pwm.ChangeDutyCycle(speed)
            print(f"[{self.name}] Setting speed: {speed}%")
            time.sleep(0.1)

    def stop_motor(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
        print(f"[{self.name}] Motor stopped.")

    def follow_sawtooth_wave(self, initial_position):
        print(f"[{self.name}] Following sawtooth wave pattern starting at {initial_position}°")
        start_time = time.time()
        while True:
            elapsed_time = time.time() - start_time
            wave_position = (elapsed_time % SAWTOOTH_PERIOD) * (330.0 / SAWTOOTH_PERIOD)
            target_position = (initial_position + wave_position) % 330
            self.target_position = target_position
            self.move_to_target()
            time.sleep(0.1)

# Parse command-line arguments for target positions
parser = argparse.ArgumentParser(description="Move motors to target positions.")
parser.add_argument("motor1_target", type=int, nargs="?", default=90, help="Target position for Motor 1 (default: 90)")
parser.add_argument("motor3_target", type=int, nargs="?", default=270, help="Target position for Motor 3 (default: 270)")
args = parser.parse_args()

motor1_target = args.motor1_target
motor3_target = args.motor3_target

try:
    motor1_controller = MotorController(
        name="Motor 1", in1=MOTOR1_IN1, in2=MOTOR1_IN2, pwm=motor1_pwm,
        adc_channel=MOTOR1_ADC_CHANNEL, target_position=motor1_target, tolerance=5
    )
    motor3_controller = MotorController(
        name="Motor 3", in1=MOTOR3_IN1, in2=MOTOR3_IN2, pwm=motor3_pwm,
        adc_channel=MOTOR3_ADC_CHANNEL, target_position=motor3_target, tolerance=5
    )

    print(f"Starting movement: Motor 1 to {motor1_target}° and Motor 3 to {motor3_target}°")
    motor1_controller.move_to_target()
    motor3_controller.move_to_target()

    # Pause and countdown after reaching target positions
    print("Orientation complete. Pausing for 5 seconds.")
    for i in range(5, 0, -1):
        print(f"Countdown: {i} seconds")
        time.sleep(1)

    # Start sawtooth wave pattern
    motor1_initial_position = motor1_target
    motor3_initial_position = motor3_target
    print("Starting sawtooth wave pattern")

    motor1_controller.follow_sawtooth_wave(motor1_initial_position)
    motor3_controller.follow_sawtooth_wave(motor3_initial_position)

except KeyboardInterrupt:
    print("Program interrupted by user")

finally:
    motor1_controller.stop_motor()
    motor3_controller.stop_motor()
    motor1_pwm.stop()
    motor3_pwm.stop()
    GPIO.cleanup()
    print("GPIO cleaned up")
