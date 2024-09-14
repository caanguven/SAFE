import time

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
        error = (self.set_point - current_value + 360) % 360
        if error > 180:
            error = error - 360  # Convert to negative value if over 180°

        # For forward-only movement, if error is negative, set to zero
        if error < 0:
            error = 0

        # Integral term with anti-windup
        self.integral += error * delta_time
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))

        # Derivative term, considering wrap-around
        delta_error = error - self.previous_error
        derivative = delta_error / delta_time

        # Compute control signal
        control_signal = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # Update state
        self.previous_error = error
        self.last_time = current_time

        return control_signal
