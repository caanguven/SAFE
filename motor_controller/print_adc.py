import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

# Constants
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023

# GPIO Pins for Motor 2
MOTOR2_IN1 = 29
MOTOR2_IN2 = 22
MOTOR2_SPD = 31
MOTOR2_ADC_CHANNEL = 1

# GPIO Pins for Motor 4
MOTOR4_IN1 = 12
MOTOR4_IN2 = 13
MOTOR4_SPD = 35
MOTOR4_ADC_CHANNEL = 3

# GPIO setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(MOTOR2_IN1, GPIO.OUT)
GPIO.setup(MOTOR2_IN2, GPIO.OUT)
GPIO.setup(MOTOR2_SPD, GPIO.OUT)
GPIO.setup(MOTOR4_IN1, GPIO.OUT)
GPIO.setup(MOTOR4_IN2, GPIO.OUT)
GPIO.setup(MOTOR4_SPD, GPIO.OUT)

# Set up PWM for motor speed control
motor2_pwm = GPIO.PWM(MOTOR2_SPD, 1000)
motor4_pwm = GPIO.PWM(MOTOR4_SPD, 1000)
motor2_pwm.start(50)  # Start Motor 2 at 50% speed
motor4_pwm.start(50)  # Start Motor 4 at 50% speed

# Set up MCP3008
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

def run_motor(in1, in2, direction="forward"):
    """Set motor direction."""
    if direction == "forward":
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    else:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)

def stop_motor(in1, in2):
    """Stop the motor."""
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)

try:
    # Start Motor 2 and Motor 4 in forward direction
    run_motor(MOTOR2_IN1, MOTOR2_IN2, "forward")
    run_motor(MOTOR4_IN1, MOTOR4_IN2, "forward")

    print("Running Motor 2 and Motor 4. Press Ctrl+C to stop.")
    while True:
        # Read ADC values for Motor 2 and Motor 4
        motor2_adc_value = mcp.read_adc(MOTOR2_ADC_CHANNEL)
        motor4_adc_value = mcp.read_adc(MOTOR4_ADC_CHANNEL)
        
        # Print ADC values
        print(f"Motor 2 ADC Value: {motor2_adc_value}")
        print(f"Motor 4 ADC Value: {motor4_adc_value}")
        
        # Delay to avoid flooding the output
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nProgram interrupted by user")

finally:
    # Stop motors and cleanup
    stop_motor(MOTOR2_IN1, MOTOR2_IN2)
    stop_motor(MOTOR4_IN1, MOTOR4_IN2)
    motor2_pwm.stop()
    motor4_pwm.stop()
    GPIO.cleanup()
    print("GPIO cleaned up")
