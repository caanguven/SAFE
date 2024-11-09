import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

# GPIO and MCP3008 setup
SPI_PORT = 0
SPI_DEVICE = 0
MOTOR_IN1 = 7
MOTOR_IN2 = 26
MOTOR_SPD = 18

# ADC channel for the potentiometer
POTENTIOMETER_CHANNEL = 0

# Set up MCP3008
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

# Set up GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(MOTOR_IN1, GPIO.OUT)
GPIO.setup(MOTOR_IN2, GPIO.OUT)
GPIO.setup(MOTOR_SPD, GPIO.OUT)

# Set up PWM for motor speed control
motor_pwm = GPIO.PWM(MOTOR_SPD, 1000)  # 1000 Hz frequency
motor_pwm.start(50)  # Start motor with 50% speed (adjust as needed)

def rotate_motor_continuous():
    GPIO.output(MOTOR_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IN2, GPIO.LOW)
    print("Motor rotating...")

def stop_motor():
    GPIO.output(MOTOR_IN1, GPIO.LOW)
    GPIO.output(MOTOR_IN2, GPIO.LOW)
    motor_pwm.ChangeDutyCycle(0)
    print("Motor stopped.")

try:
    # Start continuous motor rotation
    rotate_motor_continuous()

    # Read and print ADC values continuously
    while True:
        adc_value = mcp.read_adc(POTENTIOMETER_CHANNEL)
        print(f"ADC Value: {adc_value}")

        # Sleep for a short duration to avoid flooding the output
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program stopped by user")

finally:
    # Stop motor and cleanup GPIO
    stop_motor()
    motor_pwm.stop()
    GPIO.cleanup()
    print("GPIO cleaned up")
