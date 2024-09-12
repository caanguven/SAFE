import RPi.GPIO as GPIO
import time
import sys
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

# Pin Definitions
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

OFFSET = 5  # Allowable offset range

# PID constants (you may need to tune these)
Kp = 0.05  # Proportional gain
Ki = 0.05  # Integral gain
Kd = 0.05   # Derivative gain

# Variables for PID controller
previous_error = 0
integral = 0
last_time = time.time()

# Global variables for filtering state
filter_active = False  # Tracks if we are in a spike event
filter_count = 0       # Counts how many readings we have processed after a spike
MAX_FILTER_COUNT = 5   # Only check the next 5 values after detecting a spike
last_valid_reading = None  # Track last valid reading to reset the filter if needed
last_pot_value = None  # To track the previous potentiometer value and detect circular motion

# Millis equivalent in Python
def millis():
    return int(time.time() * 1000)

# Sawtooth wave generator
def sawtoothWave2(t, period, amplitude):
    return (t % int(period)) * (amplitude / period)

# Function to check if the reading is part of a circular transition
def is_circular_transition(current_value, previous_value):
    if previous_value is None:
        return False  # No comparison if there's no previous value

    # Check if we are transitioning from high values near 1023 to low values near 0
    if previous_value > 950 and current_value < 100:
        return True

    # Check if we are transitioning from low values near 0 to high values near 1023
    if previous_value < 100 and current_value > 950:
        return True

    return False

# Function to apply the custom spike filter
def custom_spike_filter(new_value):
    global filter_active, filter_count, last_valid_reading, last_pot_value
    
    # If the current value is part of a circular transition, skip filtering
    if last_pot_value is not None and is_circular_transition(new_value, last_pot_value):
        print(f"Circular transition detected: {last_pot_value} -> {new_value}. Skipping filter.")
        last_valid_reading = new_value
        last_pot_value = new_value
        filter_active = False  # Deactivate filter if transitioning
        return new_value

    # Reset the filter if last valid reading was not in the spike range and filtering has completed
    if filter_active and filter_count >= MAX_FILTER_COUNT:
        filter_active = False
        filter_count = 0
        print(f"Filter reset after {MAX_FILTER_COUNT} readings.")

    # Check if we are in the middle of a spike event
    if filter_active:
        filter_count += 1

        # If the value is greater than 100 and lower than 950, discard the reading
        if 100 < new_value < 950:
            print(f"Discarding invalid reading: {new_value}")
            return None  # Indicate that this value is invalid
        else:
            last_valid_reading = new_value  # Store last valid reading

        last_pot_value = new_value  # Update the last potentiometer value
        return new_value  # Return valid value
    
    # If a value is between 950 and 1023, start filtering for the next 5 readings
    if 950 <= new_value <= 1023:
        print(f"Spike detected: {new_value}, starting filter")
        filter_active = True
        filter_count = 0  # Reset the counter to begin checking next 5 readings
        last_pot_value = new_value  # Update the last potentiometer value
        return new_value  # Return the current spike value without filtering
    
    # Store the valid reading when no spike is detected
    last_valid_reading = new_value
    last_pot_value = new_value  # Update the last potentiometer value
    return new_value

# Function to map potentiometer value to degrees (0 to 360 degrees), handling dead zone
def map_potentiometer_value_with_dead_zone(value):
    # The value is from 0 to 1023. Convert it to 330 degrees for valid range
    new_value = value * (330 / 1023)
    
    # Handle dead zone from 330 to 360 degrees by interpolating the wrap-around
    if new_value > 330:
        # Interpolating the 30 degrees dead zone
        dead_zone_value = (new_value - 330) * (30 / (1023 - int(330 * (1023 / 330))))
        new_value = 360 - dead_zone_value  # Normalize to the full 360 degrees
    
    return new_value

# PID-Controller for motor control with dead zone handling
def pid_control_motor_with_dead_zone(pot_value, set_position):
    global previous_error, integral, last_time
    
    # Map the potentiometer reading to degrees, handling dead zone
    current_angle = map_potentiometer_value_with_dead_zone(pot_value)
    
    # Calculate error (with wrap-around handling for circular scale)
    error = set_position - current_angle
    
    # If the error is negative, force it to be positive (since we want forward movement only)
    if error < 0:
        error = abs(error)  # Take absolute value of error for forward-only control

    # Get the current time
    current_time = time.time()
    delta_time = current_time - last_time
    
    if delta_time >= 0.01:  # Ensure the loop doesn't update too frequently
        # Calculate integral (sum of errors over time)
        integral += error * delta_time
        
        # Calculate derivative (rate of change of error)
        derivative = (error - previous_error) / delta_time

        # Calculate the control signal using the PID controller
        control_signal = (Kp * error) + (Ki * integral) + (Kd * derivative)

        # Clamp control signal to valid PWM range (0-100%)
        control_signal = max(0, min(100, control_signal))  # Clamping to 0-100

        # If the error is within the acceptable range of the target (SET_POINT ± OFFSET)
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
            
            # Use the potentiometer value from channel 3
            pot_value = values[3]

            # Apply the custom spike filter
            filtered_pot_value = custom_spike_filter(pot_value)

            # If the filter returns None (invalid reading), skip to the next iteration
            if filtered_pot_value is None:
                continue

            # Apply sawtooth wave for dynamic set point
            current_millis = millis()
            time_for_one_cycle = 4000.0  # One cycle in milliseconds
            set_position = sawtoothWave2(current_millis, time_for_one_cycle, 360)

            # Control motor 4 based on the filtered potentiometer value and dynamic set point
            pid_control_motor_with_dead_zone(filtered_pot_value, set_position)

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
