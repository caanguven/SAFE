import RPi.GPIO as GPIO
import time
import sys

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

# Setup
GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering

# Function to safely setup GPIO pins
def setup_pin(pin_number):
    try:
        GPIO.setup(pin_number, GPIO.OUT)
    except ValueError as e:
        print(f"Error setting up GPIO pin {pin_number}: {e}")
        GPIO.cleanup()
        sys.exit(1)

# Setup pins with warning handling
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
        test_motor(M4_IN1, M4_IN2, M4_SPD)
    else:
        print("Invalid motor number. Please enter 1, 2, 3, or 4.")
        sys.exit(1)

    GPIO.cleanup()

if __name__ == "__main__":
    main()
