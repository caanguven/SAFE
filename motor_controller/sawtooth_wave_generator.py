import time

class SawtoothWaveGenerator:
    def __init__(self, period, amplitude, initial_angle):
        self.period = period
        self.amplitude = amplitude
        self.initial_angle = initial_angle
        self.start_time = self.current_millis()

    def current_millis(self):
        return int(time.time() * 1000)

    def get_set_position(self):
        t = self.current_millis() - self.start_time
        normalized_wave = (t % self.period) * (self.amplitude / self.period)
        set_position = (normalized_wave + self.initial_angle) % 360
        return set_position
