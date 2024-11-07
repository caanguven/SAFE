# ==========================
# PIDController Class - Improved
# ==========================
class PIDController:
    def __init__(self, Kp, Ki, Kd, set_point=0, integral_limit=1000):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.set_point = set_point

        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()
        self.integral_limit = integral_limit  # Max absolute value for integral term

    def compute(self, current_value):
        current_time = time.time()
        delta_time = current_time - self.last_time
        if delta_time <= 0.0:
            delta_time = 0.0001  # Avoid division by zero

        # Calculate error considering wrap-around
        error = self.set_point - current_value

        # Adjust error for minimal angle difference
        error = ((error + 180) % 360) - 180

        # Integral term with anti-windup
        self.integral += error * delta_time
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))

        # Derivative term
        derivative = (error - self.previous_error) / delta_time

        # Compute control signal
        control_signal = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # Update state
        self.previous_error = error
        self.last_time = current_time

        return control_signal

# ==========================
# MotorController Class - Improved
# ==========================
class MotorController:
    def __init__(self, motor, pid_controller, adc_reader, channel, spike_filter, config, name='Motor', initial_position=0, target_position=90):
        self.motor = motor
        self.pid_controller = pid_controller
        self.adc_reader = adc_reader
        self.channel = channel
        self.spike_filter = spike_filter
        self.config = config
        self.name = name  # For logging purposes

        # Initialization parameters
        self.initial_position = initial_position
        self.target_position = target_position
        self.initialized = False

    def pid_control_motor(self, degrees_value, set_position, initialization=False):
        OFFSET = self.config['offset']
        max_control_change = self.config['max_control_change']
        MIN_CONTROL_SIGNAL = 5  # Reduced minimum control signal to allow for finer movements

        if degrees_value is None:
            print(f"[{self.name}] degrees_value is None, cannot compute PID control.")
            self.motor.stop()
            return

        # Calculate error
        error = self.calculate_error(set_position, degrees_value)

        # Update set point in PID controller
        self.pid_controller.set_point = set_position

        # Compute control signal using PID controller
        control_signal = self.pid_controller.compute(degrees_value)

        # Apply rate limiter to control signal
        if not hasattr(self, 'last_control_signal'):
            self.last_control_signal = 0

        control_signal_change = control_signal - self.last_control_signal
        if abs(control_signal_change) > max_control_change:
            control_signal = self.last_control_signal + max_control_change * (1 if control_signal_change > 0 else -1)

        # Clamp control signal to -100 to 100
        control_signal = max(-100, min(100, control_signal))

        # During initialization, allow both forward and reverse
        if initialization:
            # Ensure a minimum control signal
            if abs(control_signal) < MIN_CONTROL_SIGNAL:
                control_signal = MIN_CONTROL_SIGNAL if control_signal > 0 else -MIN_CONTROL_SIGNAL

            # Apply control signal based on its sign
            if control_signal > 0:
                self.motor.set_speed(control_signal)
                self.motor.forward()
                dir_text = 'forward'
            else:
                self.motor.set_speed(abs(control_signal))
                self.motor.reverse()
                dir_text = 'reverse'

            print(f"[{self.name}] [Init] Moving {dir_text}: Potentiometer Value: {degrees_value:.2f}°, Error: {error:.2f}°, Control Signal: {control_signal:.2f}%, Set Position: {set_position:.2f}°")
        else:
            # Normal operation
            if abs(error) <= OFFSET:
                self.motor.set_speed(0)
                self.motor.stop()
                print(f"[{self.name}] Motor stopped at target: {degrees_value:.2f} degrees")
            else:
                # Moving normally
                speed = max(abs(control_signal), MIN_CONTROL_SIGNAL)  # Ensure minimum speed

                if control_signal > 0:
                    self.motor.set_speed(speed)
                    self.motor.forward()
                    dir_text = 'forward'
                else:
                    self.motor.set_speed(speed)
                    self.motor.reverse()
                    dir_text = 'reverse'

                print(f"[{self.name}] Moving {dir_text}: Potentiometer Value: {degrees_value:.2f}°, Error: {error:.2f}°, Control Signal: {control_signal:.2f}%, Set Position: {set_position:.2f}°")

        # Update last control signal
        self.last_control_signal = control_signal

# ==========================
# Updated Configuration
# ==========================
config = {
    'offset': 2,  # Reduced offset for better stopping precision
    'max_control_change': 3,  # Reduced max control change for finer adjustments
}

# PID constants (tune as necessary)
Kp = 0.7   # Increased Proportional gain for faster response
Ki = 0.1   # Slightly increased Integral gain to minimize steady-state error
Kd = 0.2   # Increased Derivative gain to reduce overshoot

# Rest of the setup remains the same, using Motor, PIDController, ADCReader, SpikeFilter, etc.

# ==========================
# Main Changes Summary
# ==========================
# 1. Improved PID tuning (Kp, Ki, Kd) to reduce overshoot and steady-state error.
# 2. Reduced 'offset' to ensure the motor stops closer to the exact target position.
# 3. Removed 'slowdown_threshold' to allow the motor to try and reach the target directly without slowing down.
# 4. Reduced 'max_control_change' to prevent abrupt changes and ensure finer adjustments.
# 5. Decreased 'MIN_CONTROL_SIGNAL' to allow the motor to move at a finer rate when needed.
