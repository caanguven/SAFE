import RPi.GPIO as GPIO
import time
import sys
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

# Pin Definitions
M1_IN1 = 7
M1_IN2 = 26
M1_SPD = 18

M2_IN1 = 22
M2_IN2 = 29
M2_SPD = 31

M3_IN1 = 11
M3_IN2 = 32
M3_SPD = 33

M4_IN1 = 12
M4_IN2 = 13
M4_SPD = 35

# MCP3008 setup
SPI_PORT = 0
SPI_DEVICE = 0
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

# Setup GPIO
GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering

# Function to safely setup GPIO pins
def setup_pin(pin_number):
    try:
        GPIO.setup(pin_number, GPIO.OUT)
    except ValueError as e:
        print(f"Error setting up GPIO pin {pin_number}: {e}")
        GPIO.cleanup()
        sys.exit(1)

# Setup all motor pins
setup_pin(M1_IN1)
setup_pin(M1_IN2)
setup_pin(M1_SPD)

setup_pin(M2_IN1)
setup_pin(M2_IN2)
setup_pin(M2_SPD)

setup_pin(M3_IN1)
setup_pin(M3_IN2)
setup_pin(M3_SPD)

setup_pin(M4_IN1)
setup_pin(M4_IN2)
setup_pin(M4_SPD)

# Function to map potentiometer value
def map_potentiometer_value(value):
    # Ensure the value is within the expected range of 0 to 1023
    if value < 330:
        return 0  # Treat anything below the dead space as 0 degrees
    elif value > 1023:
        return 360  # Cap the value at 360 if it goes beyond 1023 (just in case)
    else:
        # Map values from 330-1023 to 0-360 degrees
        return (value - 330) * (360 / (1023 - 330))

# Function to control motor 4 based on potentiometer value
def control_motor_4(pot_value):
    mapped_value = map_potentiometer_value(pot_value)
    print(f"Potentiometer Value: {pot_value}, Mapped Value: {mapped_value:.2f} degrees")

    if mapped_value > 0:  # If the mapped value is greater than 0, move forward
        GPIO.output(M4_IN1, GPIO.HIGH)
        GPIO.output(M4_IN2, GPIO.LOW)
        GPIO.output(M4_SPD, GPIO.HIGH)
    else:  # Stop the motor if the value is 0 or below
        print("warning the value is 0 " )

# Function to test all motors based on input number
def test_motor(IN1, IN2, SPD):
    # Forward
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(SPD, GPIO.HIGH)
    print("Motor Forward")
    time.sleep(2)

    # Stop
    GPIO.output(SPD, GPIO.LOW)
    print("Motor Stop")
    time.sleep(1)

    # Reverse
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(SPD, GPIO.HIGH)
    print("Motor Reverse")
    time.sleep(2)

    # Stop
    GPIO.output(SPD, GPIO.LOW)
    print("Motor Stop")
    time.sleep(1)

def adc_and_motor_control():
    try:
        print('Reading MCP3008 values, press Ctrl-C to quit...')
        print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*range(8)))
        print('-' * 57)

        while True:
            # Read all the ADC channel values
            values = [mcp.read_adc(i) for i in range(8)]
            
            # Print the ADC values (assuming channel 3 is for potentiometer)
            print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*values))

            # Control motor 4 based on potentiometer value
            control_motor_4(values[3])

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("Program stopped by user")
    finally:
        GPIO.cleanup()
        print("GPIO cleaned up")

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 motor_test.py <motor_number>")
        sys.exit(1)

    motor_number = int(sys.argv[1])

    if motor_number == 1:
        test_motor(M1_IN1, M1_IN2, M1_SPD)
    elif motor_number == 2:
        test_motor(M2_IN1, M2_IN2, M2_SPD)
    elif motor_number == 3:
        test_motor(M3_IN1, M3_IN2, M3_SPD)
    elif motor_number == 4:
        # Call the ADC and motor control function for motor 4
        adc_and_motor_control()
    else:
        print("Invalid motor number. Please enter 1, 2, 3, or 4.")
        sys.exit(1)

if __name__ == "__main__":
    main()
