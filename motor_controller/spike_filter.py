class SpikeFilter:
    def __init__(self):
        self.filter_active = False
        self.last_valid_reading = None

    def filter(self, new_value):
        # If the filter is active, we are in the dead zone
        if self.filter_active:
            # Discard readings between 150 and 700
            if 150 <= new_value <= 700:
                print(f"Discarding invalid reading during dead zone: {new_value}")
                return None
            else:
                # Valid reading after dead zone
                print(f"Valid reading after dead zone: {new_value}")
                self.filter_active = False
                self.last_valid_reading = new_value
                return new_value
        else:
            # Not currently filtering
            if self.last_valid_reading is not None and self.last_valid_reading > 950 and 150 <= new_value <= 700:
                # Sudden drop into dead zone detected
                print(f"Dead zone detected: last valid reading {self.last_valid_reading}, new reading {new_value}, starting filter")
                self.filter_active = True
                return None
            else:
                # Valid reading
                self.last_valid_reading = new_value
                return new_value
