import re
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
DEAD_ZONE_DEG_START = 330  # Start of dead zone in degrees
DEAD_ZONE_DEG_END = 360    # End of dead zone in degrees

# PID constants (you may need to tune these)
Kp = 0.1  # Reduced proportional gain for smoother control
Ki = 0.01  # Reduced integral gain to prevent large buildup
Kd = 0.02  # Increased derivative gain for smoother transitions

# Variables for PID controller
previous_error = 0
integral = 0
last_time = time.time()
in_dead_zone = False  # Track if the motor is in the dead zone
last_valid_control_signal = 0  # Store the last valid control signal before entering the dead zone
average_speed_before_dead_zone = 0  # To store the average speed before entering dead zone
speed_samples = []  # Keep track of a few control signals to calculate average speed

# Filter state variables
filter_active = False  # Is the filter currently active?
filter_count = 0       # How many readings we've processed since the spike
MAX_FILTER_COUNT = 5   # Maximum number of readings to process after a spike
last_valid_reading = None

# Number of samples to use for averaging speed before dead zone
NUM_SAMPLES_FOR_AVERAGE = 5

# Millis equivalent in Python
def millis():
    return int(time.time() * 1000)

# Custom filter function to discard invalid readings after a spike
def custom_spike_filter(new_value):
    global filter_active, filter_count, last_valid_reading
    
    # If filter is active, check the next few readings
    if filter_active:
        filter_count += 1
        
        # If the reading is between 300 and 950, discard it
        if 150 < new_value < 950:
            print(f"Discarding invalid reading: {new_value}")
            return None
        
        # If we've processed enough readings, disable the filter
        if filter_count >= MAX_FILTER_COUNT:
            filter_active = False
            filter_count = 0
            print(f"Filter reset after {MAX_FILTER_COUNT} readings.")
    
    # Activate the filter if we detect a spike (reading > 950)
    if new_value > 950:
        print(f"Spike detected: {new_value}, starting filter")
        filter_active = True
        filter_count = 0
        return new_value

    # If the filter is not active, return the valid reading
    last_valid_reading = new_value
    return new_value

# Function to map potentiometer value to degrees (0 to 360)
def map_potentiometer_value_to_degrees(value):
    # Potentiometer values range from 0 to 1023
    # We map this directly to 0 to 360 degrees
    return value * (360 / 1023)

# Circular error handling: Wrap errors to allow forward movement across 360 degrees
def calculate_circular_error(set_position, current_angle):
    error = set_position - current_angle

    # Adjust for circular motion (wrap-around handling)
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    
    return error


# Sawtooth wave generator that starts from the initial angle
def sawtooth_wave(t, period, amplitude, initial_angle):
    # Sawtooth wave starts from the initial angle and wraps within 0 to 360 degrees
    normalized_wave = (t % period) * (amplitude / period)
    set_position = (normalized_wave + initial_angle) % 360
    return set_position

# PID-Controller for motor control with dead zone handling
def pid_control_motor(degrees_value, set_position):
    global previous_error, integral, last_time, in_dead_zone, last_valid_control_signal, average_speed_before_dead_zone, speed_samples
    
    # Dead zone detection in degrees (330° to 360°)
    if DEAD_ZONE_DEG_START <= degrees_value <= DEAD_ZONE_DEG_END:
        if not in_dead_zone:
            # We are entering the dead zone, calculate the average speed based on previous values
            in_dead_zone = True
            average_speed_before_dead_zone = sum(speed_samples) / len(speed_samples) if speed_samples else last_valid_control_signal
            print(f"Entering dead zone. Maintaining average speed: {average_speed_before_dead_zone:.2f}%")
        
        # Continue applying the average speed calculated before entering the dead zone
        pwm.ChangeDutyCycle(average_speed_before_dead_zone)
        GPIO.output(M4_IN1, GPIO.LOW)
        GPIO.output(M4_IN2, GPIO.HIGH)
        print(f"Moving in dead zone. Control signal: {average_speed_before_dead_zone:.2f}%")
        return
    
    # Exit dead zone when degrees go back below 200°
    if in_dead_zone and degrees_value < 200:
        in_dead_zone = False
        print("Exiting dead zone. Resuming normal control.")

    # Calculate the error directly using degrees
    current_angle = degrees_value

    # Calculate circular error (with wrap-around handling for circular scale)
    error = calculate_circular_error(set_position, current_angle)

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

        # Slow down gradually as the error approaches the target
        slowdown_threshold = 15  # Degrees within which to slow down the motor

        if abs(error) <= OFFSET:
            # Stop the motor if within the target range (small error)
            GPIO.output(M4_IN1, GPIO.LOW)
            GPIO.output(M4_IN2, GPIO.LOW)
            pwm.ChangeDutyCycle(0)
            print(f"Motor stopped at target: {current_angle:.2f} degrees")
        elif abs(error) <= slowdown_threshold:
            # Reduce speed as we approach the target
            slowdown_factor = abs(error) / slowdown_threshold  # Proportional slowdown
            slow_control_signal = control_signal * slowdown_factor
            GPIO.output(M4_IN1, GPIO.LOW)
            GPIO.output(M4_IN2, GPIO.HIGH)
            pwm.ChangeDutyCycle(slow_control_signal)
            print(f"Slowing down: Potentiometer Value: {degrees_value:.2f}, Current Angle: {current_angle:.2f} degrees, Error: {error:.2f}, Control Signal: {slow_control_signal:.2f}%, Set Position: {set_position:.2f}")
        else:
            # Always move forward (ignore backward)
            GPIO.output(M4_IN1, GPIO.LOW)
            GPIO.output(M4_IN2, GPIO.HIGH)
            pwm.ChangeDutyCycle(control_signal)
            print(f"Moving forward: Potentiometer Value: {degrees_value:.2f}, Current Angle: {current_angle:.2f} degrees, Error: {error:.2f}, Control Signal: {control_signal:.2f}%, Set Position: {set_position:.2f}")
        
        # Update previous error, time, and store the last valid control signal
        previous_error = error
        last_time = current_time
        last_valid_control_signal = control_signal  # Store the last valid signal
        
        # Keep a sliding window of control signal samples to calculate average speed
        if len(speed_samples) >= NUM_SAMPLES_FOR_AVERAGE:
            speed_samples.pop(0)  # Remove the oldest sample
        speed_samples.append(control_signal)  # Add the new control signal to the list

# Main loop to read ADC values and control motor 4
def adc_and_motor_control():
    try:
        print('Reading MCP3008 values, press Ctrl-C to quit...')
        print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*range(8)))
        print('-' * 57)

        # Read the initial potentiometer value
        initial_pot_value = mcp.read_adc(3)
        initial_angle = map_potentiometer_value_to_degrees(initial_pot_value)
        print(f"Initial motor angle set to: {initial_angle:.2f} degrees")

        # Initialize time
        start_time = millis()

        while True:
            # Read all the ADC channel values
            values = [mcp.read_adc(i) for i in range(8)]
            
            # Use the potentiometer value from channel 3
            pot_value = values[3]
            
            # Apply the custom spike filter
            filtered_pot_value = custom_spike_filter(pot_value)

            # If the filter returns None (indicating an invalid reading), skip this iteration
            if filtered_pot_value is None:
                continue

            # Map the filtered potentiometer value to degrees
            degrees_value = map_potentiometer_value_to_degrees(filtered_pot_value)

            # Calculate dynamic set position based on a sawtooth wave pattern starting from initial angle
            current_time = millis()
            set_position = sawtooth_wave(current_time - start_time, 4000, 360, initial_angle)  # 4 seconds for a full cycle

            # Control motor 4 based on the potentiometer value (in degrees) and dynamic set position
            pid_control_motor(degrees_value, set_position)

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
