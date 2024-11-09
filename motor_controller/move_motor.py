import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

# Constants for SPI and ADC
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023
DEAD_ZONE_THRESHOLD = 50  # Value to detect entry/exit from dead zone

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
motor1_pwm = GPIO.PWM(MOTOR1_SPD, 1000)  # 1000 Hz frequency
motor1_pwm.start(0)  # Start with 0% speed initially

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

        # Proportional term
        proportional = self.Kp * error

        # Integral term
        self.integral += error * delta_time
        integral = self.Ki * self.integral

        # Derivative term
        derivative = self.Kd * (error - self.previous_error) / delta_time

        # Control signal
        control_signal = proportional + integral + derivative
        self.previous_error = error
        self.last_time = current_time

        return control_signal

class MotorController:
    def __init__(self, in1, in2, pwm, adc_channel, target_position, tolerance=5):
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
        self.pid = PIDController(Kp=0.8, Ki=0.1, Kd=0.05)  # PID coefficients

    def read_adc_position(self):
        # Read ADC and convert to position within 330 degrees
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

            # Use PID to compute control signal
            control_signal = self.pid.compute(error)

            # Determine motor direction based on control signal
            if control_signal > 0:
                self.set_motor_direction('forward')
            else:
                self.set_motor_direction('backward')

            # Adjust speed proportional to the control signal with a minimum threshold
            speed = min(100, max(30, abs(control_signal)))
            self.pwm.ChangeDutyCycle(speed)
            print(f"[MotorController] Setting speed: {speed}%")

            # Short delay for control loop frequency
            time.sleep(0.1)

    def stop_motor(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
        print("[MotorController] Motor stopped.")

try:
    # Initialize MotorController for Motor 1 and Motor 3
    motor1_controller = MotorController(
        in1=MOTOR1_IN1, in2=MOTOR1_IN2, pwm=motor1_pwm,
        adc_channel=MOTOR1_ADC_CHANNEL, target_position=90, tolerance=5
    )
    motor3_controller = MotorController(
        in1=MOTOR3_IN1, in2=MOTOR3_IN2, pwm=motor3_pwm,
        adc_channel=MOTOR3_ADC_CHANNEL, target_position=270, tolerance=5
    )

    # Start moving both motors toward their target positions
    print("Moving Motor 1 to 90° and Motor 3 to 270°")
    motor1_controller.move_to_target()
    motor3_controller.move_to_target()

except KeyboardInterrupt:
    print("Program interrupted by user")

finally:
    # Stop both motors and cleanup GPIO
    motor1_controller.stop_motor()
    motor3_controller.stop_motor()
    motor1_pwm.stop()
    motor3_pwm.stop()
    GPIO.cleanup()
    print("GPIO cleaned up")
