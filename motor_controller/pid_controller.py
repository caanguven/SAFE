import time

# PID constants (tune these as needed)
Kp = 0.1
Ki = 0.01
Kd = 0.02

# PID control state
previous_error = 0
integral = 0
last_time = time.time()

def pid_control(set_position, current_angle, OFFSET=5):
    global previous_error, integral, last_time

    # Calculate the error
    error = set_position - current_angle

    # Adjust for circular motion (wrap-around handling)
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360

    # Get the current time
    current_time = time.time()
    delta_time = current_time - last_time

    control_signal = 0  # Initialize control_signal

    if delta_time >= 0.01:  # Update only every 10ms
        integral += error * delta_time  # Accumulate integral
        derivative = (error - previous_error) / delta_time  # Calculate derivative

        # Calculate control signal
        control_signal = Kp * error + Ki * integral + Kd * derivative

        # Clamp control signal to valid range (0-100%)
        control_signal = max(0, min(100, control_signal))

        # Update state
        previous_error = error
        last_time = current_time

    return control_signal
