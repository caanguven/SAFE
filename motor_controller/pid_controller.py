import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()

    def calculate(self, set_position, current_angle):
        error = set_position - current_angle
        current_time = time.time()
        delta_time = current_time - self.last_time
        if delta_time >= 0.01:
            self.integral += error * delta_time
            derivative = (error - self.previous_error) / delta_time
            control_signal = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
            self.previous_error = error
            self.last_time = current_time
            return max(0, min(100, control_signal))
        return 0