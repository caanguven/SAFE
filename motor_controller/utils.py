def map_potentiometer_value_to_degrees(value):
    """Map potentiometer value (0-1023) to degrees (0-360)."""
    return value * (360 / 1023)

def custom_spike_filter(new_value, last_valid_reading, filter_active, filter_count, MAX_FILTER_COUNT=5):
    """Custom filter to discard invalid readings after a spike."""
    if filter_active:
        filter_count += 1
        if 150 < new_value < 950:
            return None, filter_active, filter_count
        if filter_count >= MAX_FILTER_COUNT:
            filter_active = False
            filter_count = 0

    if new_value > 950:
        filter_active = True
        filter_count = 0
        return new_value, filter_active, filter_count

    last_valid_reading = new_value
    return new_value, filter_active, filter_count
