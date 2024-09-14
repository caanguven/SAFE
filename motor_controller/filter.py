class Filter:
    def __init__(self, max_filter_count):
        self.filter_active = False
        self.filter_count = 0
        self.last_valid_reading = None
        self.max_filter_count = max_filter_count

    def apply(self, new_value):
        if self.filter_active:
            self.filter_count += 1
            if 150 < new_value < 950:
                return None
            if self.filter_count >= self.max_filter_count:
                self.filter_active = False
                self.filter_count = 0
        if new_value > 950:
            self.filter_active = True
            self.filter_count = 0
            return new_value
        self.last_valid_reading = new_value
        return new_value