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
GPIO.setup(M4_IN1, GPIO.OUT)
GPIO.setup(M4_IN2, GPIO.OUT)
GPIO.setup(M4_SPD, GPIO.OUT)

# Setup PWM for motor speed control on M4_SPD
pwm = GPIO.PWM(M4_SPD, 1000)  # Set PWM frequency to 1kHz
pwm.start(0)  # Start with 0% duty cycle (motor off)

# Set point for the motor (desired angle)
SET_POINT = 200  # Target degrees
OFFSET = 5  # Allowable offset range

# Proportional gain (Kp), you can adjust this value
Kp = 1.0

# Function to map potentiometer value to degrees
def map_potentiometer_value(value):
    if value < 330:
        return 0  # Treat anything below the dead space as 0 degrees
    elif value > 1023:
        return 360  # Cap the value at 360 if it goes beyond 1023
    else:
        return (value - 330) * (360 / (1023 - 330))  # Map 330-1023 to 0-360 degrees

# P-Controller for motor control
def p_control_motor_4(pot_value):
    # Map the potentiometer reading to degrees
    current_angle = map_potentiometer_value(pot_value)
    
    # Calculate the error
    error = SET_POINT - current_angle

    # Calculate the control signal using the P controller
    control_signal = Kp * abs(error)

    # Clamp control signal to valid PWM range (0-100%)
    control_signal = max(0, min(100, control_signal))  # Clamping to 0-100

    # Check if the current angle is within the acceptable range of the target (SET_POINT ± OFFSET)
    if SET_POINT - OFFSET <= current_angle <= SET_POINT + OFFSET:
        # Stop the motor if within the target range
        GPIO.output(M4_IN1, GPIO.LOW)
        GPIO.output(M4_IN2, GPIO.LOW)
        pwm.ChangeDutyCycle(0)
        print(f"Motor stopped at target: {current_angle:.2f} degrees")
    elif current_angle < SET_POINT - OFFSET:
        # Move forward if the current angle is below the lower bound of the target
        GPIO.output(M4_IN1, GPIO.HIGH)
        GPIO.output(M4_IN2, GPIO.LOW)
        pwm.ChangeDutyCycle(control_signal)
        print(f"Moving forward: Potentiometer Value: {pot_value}, Current Angle: {current_angle:.2f} degrees, Error: {error:.2f}, Control Signal: {control_signal:.2f}%")
    elif current_angle > SET_POINT + OFFSET:
        # Reverse if the current angle is above the upper bound of the target
        GPIO.output(M4_IN1, GPIO.LOW)
        GPIO.output(M4_IN2, GPIO.HIGH)
        pwm.ChangeDutyCycle(control_signal)
        print(f"Moving backward: Potentiometer Value: {pot_value}, Current Angle: {current_angle:.2f} degrees, Error: {error:.2f}, Control Signal: {control_signal:.2f}%")

# Main loop to read ADC values and control motor 4
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

            # Control motor 4 based on potentiometer value (channel 3)
            p_control_motor_4(values[3])

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Program stopped by user")
    finally:
        pwm.stop()
        GPIO.cleanup()
        print("GPIO cleaned up")

# Main function to run the test for motor 4
def main():
    adc_and_motor_control()

if __name__ == "__main__":
    main()
