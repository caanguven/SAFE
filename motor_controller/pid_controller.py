import time

class PIDController:
    def __init__(self, Kp, Ki, Kd, set_point=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.set_point = set_point

        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, current_value):
        current_time = time.time()
        delta_time = current_time - self.last_time
        if delta_time <= 0.0:
            delta_time = 0.0001  # Avoid division by zero

        error = self.set_point - current_value
        self.integral += error * delta_time
        derivative = (error - self.previous_error) / delta_time

        # Compute control signal
        control_signal = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # Update state
        self.previous_error = error
        self.last_time = current_time

        return control_signal
