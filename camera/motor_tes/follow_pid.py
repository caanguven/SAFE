import RPi.GPIO as GPIO
import time
import sys
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import collections

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

# PID constants (you may need to tune these)
Kp = 0.2  # Proportional gain
Ki = 0.05  # Integral gain
Kd = 0.1   # Derivative gain

# Variables for PID controller
previous_error = 0
integral = 0
last_time = time.time()

# Deque to hold the moving window for circular median filtering
WINDOW_SIZE = 5
adc_values = collections.deque(maxlen=WINDOW_SIZE)

# Global SET_POINT variable
SET_POINT = 0

# Function to map potentiometer value to degrees (0 to 330 degrees mapped from 0 to 1023)
def map_potentiometer_value(value):
    new_value = value * (330 / 1023)
    if new_value > 330:
        new_value = 360
    return new_value

# PID-Controller for motor control with forward-only movement
def pid_control_motor_4(pot_value):
    global previous_error, integral, last_time, SET_POINT
    
    # Map the potentiometer reading to degrees
    current_angle = map_potentiometer_value(pot_value)
    
    # Calculate error with circular wrap-around handling
    error = SET_POINT - current_angle
    
    # Check if the error crosses the circular boundary
    if current_angle > SET_POINT:
        error = (360 - current_angle) + SET_POINT
    
    current_time = time.time()
    delta_time = current_time - last_time
    
    if delta_time >= 0.01:
        # Calculate integral (sum of errors over time)
        integral += error * delta_time
        
        # Calculate derivative (rate of change of error)
        derivative = (error - previous_error) / delta_time

        # Calculate the control signal using the PID controller
        control_signal = (Kp * error) + (Ki * integral) + (Kd * derivative)

        # Clamp control signal to valid PWM range (0-100%)
        control_signal = max(0, min(100, control_signal))

        # Check if the current angle is within the acceptable range of the target (SET_POINT ± OFFSET)
        OFFSET = 5  # Allowable offset range
        if abs(error) <= OFFSET:
            GPIO.output(M4_IN1, GPIO.LOW)
            GPIO.output(M4_IN2, GPIO.LOW)
            pwm.ChangeDutyCycle(0)
            print(f"Motor stopped at target: {current_angle:.2f} degrees")
        else:
            GPIO.output(M4_IN1, GPIO.LOW)
            GPIO.output(M4_IN2, GPIO.HIGH)
            pwm.ChangeDutyCycle(control_signal)
            print(f"Moving forward: Potentiometer Value: {pot_value}, Current Angle: {current_angle:.2f} degrees, Error: {error:.2f}, Control Signal: {control_signal:.2f}%")
        
        previous_error = error
        last_time = current_time

# Sawtooth wave generator function
def sawtooth_wave(t, period, amplitude):
    """Generate a sawtooth wave given time `t`, period, and amplitude."""
    return (t % period) * (amplitude / period)

# Main loop to read ADC values, generate a sawtooth wave, and control motor 4
def adc_and_motor_control():
    try:
        print('Reading MCP3008 values, press Ctrl-C to quit...')
        print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*range(8)))
        print('-' * 57)

        period = 10  # Period of the sawtooth wave (in seconds)
        amplitude = 360  # Amplitude of the sawtooth wave (in degrees)

        start_time = time.time()

        while True:
            # Calculate elapsed time
            elapsed_time = time.time() - start_time

            # Generate a sawtooth wave setpoint
            sawtooth_setpoint = sawtooth_wave(elapsed_time, period, amplitude)

            # Set the global SET_POINT to follow the sawtooth wave
            global SET_POINT
            SET_POINT = sawtooth_setpoint

            # Read all the ADC channel values
            values = [mcp.read_adc(i) for i in range(8)]
            
            # Control motor 4 based on the raw potentiometer value
            pid_control_motor_4(values[3])

            print(f"Sawtooth Setpoint: {SET_POINT:.2f} degrees")
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
