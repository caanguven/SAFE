import time

class SawtoothWaveGenerator:
    def __init__(self, period_ms, amplitude_degrees, direction='forward'):
        self.period_ms = period_ms
        self.amplitude_degrees = amplitude_degrees
        self.start_time = 0
        self.direction = direction  # 'forward' or 'reverse'

    def current_millis(self):
        return int(time.time() * 1000)

    def get_set_position(self):
        elapsed_time = (self.current_millis() - self.start_time) % self.period_ms
        normalized_wave = (elapsed_time / self.period_ms) * self.amplitude_degrees

        if self.direction == 'forward':
            return normalized_wave % 360
        elif self.direction == 'reverse':
            return (360 - normalized_wave) % 360
        else:
            raise ValueError("Invalid direction. Must be 'forward' or 'reverse'.")
