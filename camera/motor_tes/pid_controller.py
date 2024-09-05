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

# Command-line argument handling for SET_POINT
if len(sys.argv) != 2:
    print("Usage: python pid.py <set_point>")
    sys.exit(1)

try:
    SET_POINT = int(sys.argv[1])
    if SET_POINT < 0 or SET_POINT > 360:
        raise ValueError("SET_POINT must be between 0 and 360 degrees")
except ValueError as e:
    print(f"Error: {e}")
    sys.exit(1)

OFFSET = 5  # Allowable offset range

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

# Function to calculate the circular distance between two points on a circular scale
def circular_distance(value1, value2, max_value=1023):
    diff = abs(value1 - value2)
    return min(diff, max_value - diff)

# Function to apply a circular median filter on ADC values
def circular_median_filter(new_value):
    adc_values.append(new_value)
    
    # Sort values based on circular distance to the middle value
    sorted_values = sorted(adc_values, key=lambda x: circular_distance(x, adc_values[len(adc_values) // 2]))
    
    # Return the circular median
    return sorted_values[len(sorted_values) // 2]

# Function to apply a custom filter on ADC values
# Function to apply a custom filter on ADC values
# Global variable to keep track of filtering state
filter_active = False
filter_count = 0

# Function to apply a custom filter on ADC values
def custom_filter(new_value):
    global filter_active, filter_count

    # If we're filtering (i.e., after a reading > 960), process the next readings
    if filter_active:
        # Continue filtering for the next 3 readings
        if filter_count < 3:
            filter_count += 1
            if new_value <= 200:
                return new_value  # If valid, return the filtered value
            else:
                return 200  # Otherwise, return a fixed value (e.g., 200) for values > 200
        else:
            # Reset filter after 3 readings
            filter_active = False
            filter_count = 0

    # If the current value is greater than 960, activate the filter
    if new_value > 960:
        filter_active = True
        filter_count = 0  # Reset the count to start filtering the next 3 readings
        return new_value  # Return the current reading without filtering

    # Otherwise, return the unfiltered value
    return new_value




# Function to map potentiometer value to degrees (0 to 330 degrees mapped from 0 to 1023)
def map_potentiometer_value(value):
    # The value is from 0 to 1023. Convert it to 330 degrees 
    new_value = value * (330 / 1023)
    # If the new_value is bigger than 330, normalize it to 360
    if new_value > 330:
        new_value = 360
    return new_value

# PID-Controller for motor control with forward-only movement
# PID-Controller for motor control with forward-only movement
def pid_control_motor_4(pot_value):
    global previous_error, integral, last_time
    
    # Map the potentiometer reading to degrees
    current_angle = map_potentiometer_value(pot_value)
    
    # Calculate error with circular wrap-around handling
    error = SET_POINT - current_angle
    
    # Check if the error crosses the circular boundary
    if current_angle > SET_POINT:
        # If we're wrapping around the 360-degree boundary, adjust error
        error = (360 - current_angle) + SET_POINT
    
    # Get the current time
    current_time = time.time()
    delta_time = current_time - last_time
    
    if delta_time >= 0.01:  # Avoid division by zero and ensure the loop doesn't update too frequently
        # Calculate integral (sum of errors over time)
        integral += error * delta_time
        
        # Calculate derivative (rate of change of error)
        derivative = (error - previous_error) / delta_time

        # Calculate the control signal using the PID controller
        control_signal = (Kp * error) + (Ki * integral) + (Kd * derivative)

        # Clamp control signal to valid PWM range (0-100%)
        control_signal = max(0, min(100, control_signal))  # Clamping to 0-100

        # Check if the current angle is within the acceptable range of the target (SET_POINT ± OFFSET)
        if abs(error) <= OFFSET:
            # Stop the motor if within the target range
            GPIO.output(M4_IN1, GPIO.LOW)
            GPIO.output(M4_IN2, GPIO.LOW)
            pwm.ChangeDutyCycle(0)
            print(f"Motor stopped at target: {current_angle:.2f} degrees")
        else:
            # Always move forward (ignore backward)
            GPIO.output(M4_IN1, GPIO.LOW)
            GPIO.output(M4_IN2, GPIO.HIGH)
            pwm.ChangeDutyCycle(control_signal)
            print(f"Moving forward: Potentiometer Value: {pot_value}, Current Angle: {current_angle:.2f} degrees, Error: {error:.2f}, Control Signal: {control_signal:.2f}%")
        
        # Update previous error and time
        previous_error = error
        last_time = current_time

# Main loop to read ADC values and control motor 4
def adc_and_motor_control():
    try:
        print('Reading MCP3008 values, press Ctrl-C to quit...')
        print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*range(8)))
        print('-' * 57)

        while True:
            # Read all the ADC channel values
            values = [mcp.read_adc(i) for i in range(8)]
            
            # Apply the circular median filter to the raw ADC value from channel 3 (potentiometer)
            filtered_pot_value = circular_median_filter(values[3])

            filtered_pot_value2 = custom_filter(values[3])

            # Check if the custom filter returned None (no valid reading), if so skip this iteration
            if filtered_pot_value2 is None:
                print("No valid reading after filtering, skipping motor control.")
                continue


            # Control motor 4 based on the filtered potentiometer value
            pid_control_motor_4(filtered_pot_value2)

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
