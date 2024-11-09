import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import math

# Constants
SPI_PORT = 0
SPI_DEVICE = 0
MOTOR_IN1 = 7
MOTOR_IN2 = 26
MOTOR_SPD = 18
POTENTIOMETER_CHANNEL = 0

# Motor and ADC configuration
DEAD_ZONE_THRESHOLD = 900  # Threshold to detect dead zone based on ADC jump
DEAD_ZONE_START = 990  # Approximate ADC value before entering dead zone
DEAD_ZONE_END = 30    # Approximate ADC value after leaving dead zone

# Set up MCP3008
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

# Set up GPIO for motor
GPIO.setmode(GPIO.BOARD)
GPIO.setup(MOTOR_IN1, GPIO.OUT)
GPIO.setup(MOTOR_IN2, GPIO.OUT)
GPIO.setup(MOTOR_SPD, GPIO.OUT)

# Set up PWM for motor control
motor_pwm = GPIO.PWM(MOTOR_SPD, 1000)  # 1000 Hz frequency
motor_pwm.start(0)  # Start motor with 0% duty cycle (motor off)


class MotorController:
    def __init__(self, in1_pin, in2_pin, pwm_pin, adc_channel, pwm_frequency=1000):
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.pwm_pin = pwm_pin
        self.adc_channel = adc_channel
        self.current_position = 0.0
        self.last_adc_value = None
        self.in_dead_zone = False
        self.direction = "forward"

        # Setup motor GPIO
        GPIO.setup(self.in1_pin, GPIO.OUT)
        GPIO.setup(self.in2_pin, GPIO.OUT)
        GPIO.setup(self.pwm_pin, GPIO.OUT)

        # Setup PWM for motor speed control
        self.pwm = GPIO.PWM(self.pwm_pin, pwm_frequency)
        self.pwm.start(0)

    def read_adc(self):
        return mcp.read_adc(self.adc_channel)

    def move_to_position(self, target_position, tolerance=5):
        while True:
            # Read current ADC value and convert it to degrees within the 0-330° range
            adc_value = self.read_adc()
            degrees = (adc_value / 1023.0) * 330.0

            # Check if we are entering or exiting the dead zone
            if self.is_in_dead_zone(adc_value):
                # Start estimating position while in dead zone based on direction and timing
                if self.direction == "forward":
                    self.current_position += 330 / 1023 * DEAD_ZONE_THRESHOLD
                elif self.direction == "backward":
                    self.current_position -= 330 / 1023 * DEAD_ZONE_THRESHOLD
                print(f"In Dead Zone - Estimated Position: {self.current_position:.2f}")
            else:
                # Update position from ADC if not in dead zone
                self.current_position = degrees
                self.in_dead_zone = False  # Reset dead zone flag

            # Calculate position error
            error = target_position - self.current_position

            # Check if within tolerance
            if abs(error) <= tolerance:
                self.stop()
                print(f"Reached target position: {self.current_position:.2f}° within tolerance.")
                break

            # Adjust motor speed and direction based on error
            self.set_motor_direction_and_speed(error)

            print(f"Target: {target_position}°, Current: {self.current_position:.2f}°, Error: {error:.2f}")
            time.sleep(0.1)  # Control loop delay

    def is_in_dead_zone(self, adc_value):
        """Detect if the ADC value is in the dead zone range"""
        if self.last_adc_value is not None:
            if abs(adc_value - self.last_adc_value) > DEAD_ZONE_THRESHOLD:
                self.in_dead_zone = True
                self.last_adc_value = adc_value  # Update for wraparound detection
                return True
        self.last_adc_value = adc_value
        return False

    def set_motor_direction_and_speed(self, error):
        """Set the motor direction and speed based on error."""
        speed = min(100, max(10, abs(error) * 0.5))  # Scale speed based on error

        # Adjust motor direction based on error sign
        if error > 0:
            self.forward(speed)
            self.direction = "forward"
        else:
            self.reverse(speed)
            self.direction = "backward"

    def forward(self, duty_cycle):
        GPIO.output(self.in1_pin, GPIO.HIGH)
        GPIO.output(self.in2_pin, GPIO.LOW)
        self.pwm.ChangeDutyCycle(duty_cycle)
        print(f"Motor moving forward at {duty_cycle}% duty cycle")

    def reverse(self, duty_cycle):
        GPIO.output(self.in1_pin, GPIO.LOW)
        GPIO.output(self.in2_pin, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(duty_cycle)
        print(f"Motor moving in reverse at {duty_cycle}% duty cycle")

    def stop(self):
        GPIO.output(self.in1_pin, GPIO.LOW)
        GPIO.output(self.in2_pin, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
        print("Motor stopped")


# Main program
try:
    # Create motor controller instance
    motor_controller = MotorController(MOTOR_IN1, MOTOR_IN2, MOTOR_SPD, POTENTIOMETER_CHANNEL)

    # Target position to move to
    target_position = 165  # Target midpoint of 330 degrees

    # Move motor to the target position
    motor_controller.move_to_position(target_position)

except KeyboardInterrupt:
    print("Program interrupted by user")

finally:
    # Stop motor and clean up GPIO
    motor_controller.stop()
    motor_pwm.stop()
    GPIO.cleanup()
    print("GPIO cleaned up")
