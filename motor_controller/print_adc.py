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

# GPIO setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(MOTOR2_IN1, GPIO.OUT)
GPIO.setup(MOTOR2_IN2, GPIO.OUT)
GPIO.setup(MOTOR2_SPD, GPIO.OUT)

# Set up PWM for motor speed control
motor2_pwm = GPIO.PWM(MOTOR2_SPD, 1000)
motor2_pwm.start(40)  # Start Motor 2 at 10% speed for slow movement

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
    # Start Motor 2 in forward direction
    run_motor(MOTOR2_IN1, MOTOR2_IN2, "forward")

    print("Running Motor 2 slowly. Press Ctrl+C to stop.")
    while True:
        # Read ADC value for Motor 2
        motor2_adc_value = mcp.read_adc(MOTOR2_ADC_CHANNEL)
        
        # Print ADC value
        print(f"Motor 2 ADC Value: {motor2_adc_value}")
        
        # Delay to avoid flooding the output
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nProgram interrupted by user")

finally:
    # Stop motor and cleanup
    stop_motor(MOTOR2_IN1, MOTOR2_IN2)
    motor2_pwm.stop()
    GPIO.cleanup()
    print("GPIO cleaned up") 
