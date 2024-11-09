import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import statistics

# Constants
SPI_PORT = 0
SPI_DEVICE = 0
MOTOR_IN1 = 7
MOTOR_IN2 = 26
MOTOR_SPD = 18
POTENTIOMETER_CHANNEL = 0

# Threshold for detecting erroneous jumps
JUMP_THRESHOLD = 150

# Set up MCP3008
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

# Set up GPIO for motor
GPIO.setmode(GPIO.BOARD)
GPIO.setup(MOTOR_IN1, GPIO.OUT)
GPIO.setup(MOTOR_IN2, GPIO.OUT)
GPIO.setup(MOTOR_SPD, GPIO.OUT)

# Set up PWM for motor control
motor_pwm = GPIO.PWM(MOTOR_SPD, 1000)  # 1000 Hz frequency
motor_pwm.start(50)  # Start motor at 50% speed (can adjust)

class ADCReader:
    def __init__(self, channel, num_samples=5):
        self.channel = channel
        self.num_samples = num_samples
        self.readings = []
        self.previous_value = None

    def read_and_filter_adc(self):
        # Read the raw ADC value
        adc_value = mcp.read_adc(self.channel)

        # Initialize previous value for first run
        if self.previous_value is None:
            self.previous_value = adc_value

        # Check for sudden jump
        if abs(adc_value - self.previous_value) > JUMP_THRESHOLD:
            print(f"[Warning] Sudden jump detected in ADC value: {adc_value}, ignoring.")
            return None  # Discard this reading

        # Store reading if it's within reasonable range
        self.readings.append(adc_value)
        if len(self.readings) > self.num_samples:
            self.readings.pop(0)

        # Update previous value for next comparison
        self.previous_value = adc_value

        # Calculate moving average
        avg_value = int(statistics.mean(self.readings))
        print(f"[ADCReader] ADC Value: {adc_value}, Filtered Average: {avg_value}")
        return avg_value

# Motor functions
def rotate_motor_continuous(direction="forward"):
    if direction == "forward":
        GPIO.output(MOTOR_IN1, GPIO.LOW)
        GPIO.output(MOTOR_IN2, GPIO.HIGH)
    elif direction == "backward":
        GPIO.output(MOTOR_IN1, GPIO.HIGH)
        GPIO.output(MOTOR_IN2, GPIO.LOW)
    print(f"Motor rotating {direction}...")

def stop_motor():
    GPIO.output(MOTOR_IN1, GPIO.LOW)
    GPIO.output(MOTOR_IN2, GPIO.LOW)
    motor_pwm.ChangeDutyCycle(0)
    print("Motor stopped.")

# Main loop for testing ADC reading with filtering
try:
    adc_reader = ADCReader(channel=POTENTIOMETER_CHANNEL)
    
    # Start motor rotation
    rotate_motor_continuous(direction="forward")
    
    # Read and filter ADC values
    while True:
        filtered_value = adc_reader.read_and_filter_adc()
        if filtered_value is not None:
            print(f"Filtered ADC Value: {filtered_value}")
        
        # Sleep briefly to avoid flooding the output
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program interrupted by user")

finally:
    # Stop motor and clean up GPIO
    stop_motor()
    motor_pwm.stop()
    GPIO.cleanup()
    print("GPIO cleaned up")
