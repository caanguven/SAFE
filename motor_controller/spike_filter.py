class SpikeFilter:
    def __init__(self):
        self.filter_active = False
        self.last_valid_reading = None

    def filter(self, new_value):
        if self.filter_active:
            # We are currently filtering invalid readings in the dead zone
            if new_value <= 150:
                # Valid reading after the dead zone
                print(f"Valid reading after dead zone: {new_value}")
                self.filter_active = False
                self.last_valid_reading = new_value
                return new_value
            else:
                # Discard readings between 150 and 950
                print(f"Discarding invalid reading during dead zone: {new_value}")
                return None

        else:
            # Not currently filtering
            if new_value > 950:
                # Spike detected, entering dead zone
                print(f"Spike detected: {new_value}, entering dead zone, starting filter")
                self.filter_active = True
                return None  # Optionally, you can return new_value if you need to process the spike
            else:
                # Valid reading
                self.last_valid_reading = new_value
                return new_value
