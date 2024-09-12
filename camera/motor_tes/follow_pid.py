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
DEAD_ZONE_START = 950  # Start of dead zone in potentiometer value (corresponds to 330 degrees)
DEAD_ZONE_END = 1023   # End of dead zone in potentiometer value (corresponds to 360 degrees)
RESUME_ZONE = 200      # Resume zone (below this, the motor should start reading normally again)

# PID constants (you may need to tune these)
Kp = 0.05  # Proportional gain
Ki = 0.05  # Integral gain
Kd = 0.05   # Derivative gain

# Variables for PID controller
previous_error = 0
integral = 0
last_time = time.time()
in_dead_zone = False  # Track if the motor is in the dead zone
last_valid_control_signal = 0  # Store the last valid control signal before entering the dead zone

# Millis equivalent in Python
def millis():
    return int(time.time() * 1000)

# Sawtooth wave generator for dynamic set position (between 0 and 1023)
def sawtooth_wave(t, period, amplitude=1023):
    return (t % period) * (amplitude / period)

# Circular error handling: Wrap errors to allow forward movement across 360 degrees
def calculate_circular_error(set_position, current_angle):
    error = set_position - current_angle
    
    # If error is negative, wrap it to simulate forward movement across 360 degrees
    if error < 0:
        error += 1023
    
    return error

# PID-Controller for motor control with dead zone handling
def pid_control_motor(pot_value, set_position):
    global previous_error, integral, last_time, in_dead_zone, last_valid_control_signal
    
    # Dead zone detection
    if DEAD_ZONE_START <= pot_value <= DEAD_ZONE_END:
        if not in_dead_zone:
            in_dead_zone = True
            print("Entering dead zone. Maintaining last control signal.")
        # Continue applying the last control signal (the motor keeps moving as per the previous command)
        pwm.ChangeDutyCycle(last_valid_control_signal)
        GPIO.output(M4_IN1, GPIO.LOW)
        GPIO.output(M4_IN2, GPIO.HIGH)
        print(f"Moving in dead zone. Last control signal: {last_valid_control_signal:.2f}%")
        return
    
    # Exit dead zone when the potentiometer value drops below the resume zone
    if in_dead_zone and pot_value < RESUME_ZONE:
        in_dead_zone = False
        print("Exiting dead zone. Resuming normal control.")

    # Calculate the error directly using potentiometer value as the angle
    current_angle = pot_value  # No need to map, just use the raw potentiometer value

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
            print(f"Moving forward: Potentiometer Value: {pot_value}, Current Angle: {current_angle:.2f}, Error: {error:.2f}, Control Signal: {control_signal:.2f}%, Set Position: {set_position:.2f}")
        
        # Update previous error, time, and store the last valid control signal
        previous_error = error
        last_time = current_time
        last_valid_control_signal = control_signal  # Store the last valid signal

# Main loop to read ADC values and control motor 4
def adc_and_motor_control():
    try:
        print('Reading MCP3008 values, press Ctrl-C to quit...')
        print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*range(8)))
        print('-' * 57)

        # Initialize time
        start_time = millis()

        while True:
            # Read all the ADC channel values
            values = [mcp.read_adc(i) for i in range(8)]
            
            # Use the potentiometer value from channel 3
            pot_value = values[3]

            # Calculate dynamic set position based on a sawtooth wave pattern
            current_time = millis()
            set_position = sawtooth_wave(current_time - start_time, 4000)  # 4 seconds for a full cycle

            # Control motor 4 based on the potentiometer value and dynamic set position
            pid_control_motor(pot_value, set_position)

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
