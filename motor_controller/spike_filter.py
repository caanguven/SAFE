class SpikeFilter:
    def __init__(self, max_filter_count=5):
        self.filter_active = False
        self.filter_count = 0
        self.MAX_FILTER_COUNT = max_filter_count
        self.last_valid_reading = None

    def filter(self, new_value):
        # If filter is active, check the next few readings
        if self.filter_active:
            self.filter_count += 1

            # If the reading is between 150 and 950, discard it
            if 150 < new_value < 950:
                print(f"Discarding invalid reading: {new_value}")
                return None

            # If we've processed enough readings, disable the filter
            if self.filter_count >= self.MAX_FILTER_COUNT:
                self.filter_active = False
                self.filter_count = 0
                print(f"Filter reset after {self.MAX_FILTER_COUNT} readings.")

        # Activate the filter if we detect a spike (reading > 950)
        if new_value > 950:
            print(f"Spike detected: {new_value}, starting filter")
            self.filter_active = True
            self.filter_count = 0
            return new_value

        # If the filter is not active, return the valid reading
        self.last_valid_reading = new_value
        return new_value
