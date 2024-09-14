import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
from motor_pins import MOTOR_PINS
from motor_controller import MotorController
from pid_controller import PIDController
from filter import Filter

SPI_PORT = 0
SPI_DEVICE = 0
OFFSET = 5
DEAD_ZONE_DEG_START = 330
DEAD_ZONE_DEG_END = 360
Kp = 0.5  # Increased proportional gain
Ki = 0.05  # Increased integral gain
Kd = 0.1  # Increased derivative gain
MAX_FILTER_COUNT = 5
NUM_SAMPLES_FOR_AVERAGE = 5

previous_error = 0
integral = 0
last_time = time.time()
in_dead_zone = False
last_valid_control_signal = 0
average_speed_before_dead_zone = 0
speed_samples = []

def millis():
    return int(time.time() * 1000)

def map_potentiometer_value_to_degrees(value):
    return value * (330 / 1023)

def sawtooth_wave(t, period, amplitude, initial_angle):
    normalized_wave = (t % period) * (amplitude / period)
    set_position = (normalized_wave + initial_angle) % 360
    return set_position

def calculate_circular_error(set_position, current_angle):
    error = set_position - current_angle
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    return error

def pid_control_motor(degrees_value, set_position, motor, pid):
    global previous_error, integral, last_time, in_dead_zone, last_valid_control_signal, average_speed_before_dead_zone, speed_samples
    
    # Dead zone detection in degrees (330° to 360°)
    if DEAD_ZONE_DEG_START <= degrees_value <= DEAD_ZONE_DEG_END:
        if not in_dead_zone:
            # We are entering the dead zone, calculate the average speed based on previous values
            in_dead_zone = True
            average_speed_before_dead_zone = sum(speed_samples) / len(speed_samples) if speed_samples else last_valid_control_signal
            print(f"Entering dead zone. Maintaining average speed: {average_speed_before_dead_zone:.2f}%")
        
        # Continue applying the average speed calculated before entering the dead zone
        motor.set_speed(average_speed_before_dead_zone)
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
        slowdown_threshold = 20  # Degrees within which to slow down the motor

        if abs(error) <= OFFSET:
            # Stop the motor if within the target range (small error)
            motor.stop()
            print(f"Motor stopped at target: {current_angle:.2f} degrees")
        elif abs(error) <= slowdown_threshold:
            # Reduce speed as we approach the target
            slowdown_factor = abs(error) / slowdown_threshold  # Proportional slowdown
            slow_control_signal = control_signal * slowdown_factor
            motor.set_speed(slow_control_signal)
            print(f"Slowing down: Potentiometer Value: {degrees_value:.2f}, Current Angle: {current_angle:.2f} degrees, Error: {error:.2f}, Control Signal: {slow_control_signal:.2f}%, Set Position: {set_position:.2f}")
        else:
            # Always move forward (ignore backward)
            motor.set_speed(control_signal)
            print(f"Moving forward: Potentiometer Value: {degrees_value:.2f}, Current Angle: {current_angle:.2f} degrees, Error: {error:.2f}, Control Signal: {control_signal:.2f}%, Set Position: {set_position:.2f}")
        
        # Update previous error, time, and store the last valid control signal
        previous_error = error
        last_time = current_time
        last_valid_control_signal = control_signal  # Store the last valid signal
        
        # Keep a sliding window of control signal samples to calculate average speed
        if len(speed_samples) >= NUM_SAMPLES_FOR_AVERAGE:
            speed_samples.pop(0)  # Remove the oldest sample
        speed_samples.append(control_signal)  # Add the new control signal to the list

def main():
    mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))
    motor_pins = MOTOR_PINS[4]  # Example for motor 4
    motor = MotorController(motor_pins['IN1'], motor_pins['IN2'], motor_pins['SPD'])
    pid = PIDController(Kp, Ki, Kd)
    filter = Filter(MAX_FILTER_COUNT)

    try:
        initial_pot_value = mcp.read_adc(3)
        initial_angle = map_potentiometer_value_to_degrees(initial_pot_value)
        start_time = millis()

        # Move motor to 180 degrees at the start
        target_angle = 180
        while True:
            pot_value = mcp.read_adc(3)
            filtered_pot_value = filter.apply(pot_value)
            if filtered_pot_value is None:
                continue
            degrees_value