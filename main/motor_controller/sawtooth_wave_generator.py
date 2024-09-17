import time

class SawtoothWaveGenerator:
    def __init__(self, period, amplitude, initial_angle, direction='forward'):
        self.period = period
        self.amplitude = amplitude
        self.initial_angle = initial_angle
        self.direction = direction  # 'forward' or 'reverse'
        self.start_time = self.current_millis()

    def current_millis(self):
        return int(time.time() * 1000)

    def get_set_position(self):
        t = self.current_millis() - self.start_time
        normalized_wave = (t % self.period) * (self.amplitude / self.period)
        
        if self.direction == 'forward':
            set_position = (normalized_wave + self.initial_angle) % 360
        elif self.direction == 'reverse':
            set_position = (360 - (normalized_wave + self.initial_angle)) % 360
        else:
            raise ValueError("Invalid direction. Must be 'forward' or 'reverse'.")
        
        return set_position

    def set_direction(self, direction):
        if direction not in ['forward', 'reverse']:
            raise ValueError("Direction must be 'forward' or 'reverse'.")
        self.direction = direction
