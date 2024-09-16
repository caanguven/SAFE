import time

class MotorController:
    def __init__(self, motor, pid_controller, adc_reader, channel, spike_filter, sawtooth_generator, config, name='Motor'):
        self.motor = motor
        self.pid_controller = pid_controller
        self.adc_reader = adc_reader
        self.channel = channel
        self.spike_filter = spike_filter
        self.sawtooth_generator = sawtooth_generator
        self.config = config
        self.name = name  # For logging purposes

        # Variables for state
        self.in_dead_zone = False
        self.last_valid_control_signal = 0
        self.average_speed_before_dead_zone = 0
        self.speed_samples = []
        self.dead_zone_start_time = None
        self.estimated_position = None
        self.last_degrees_value = None
        self.last_time = time.time()  # Initialize last_time

    def map_potentiometer_value_to_degrees(self, value):
        # Potentiometer values range from 0 to 1023
        # Map to 0 to 330 degrees (since the potentiometer only reads up to 330°)
        return value * (330 / 1023)

    def calculate_error(self, set_position, current_angle):
        # Calculate the forward-only error, handling wrap-around
        error = (set_position - current_angle + 360) % 360
        if error > 180:
            error = error - 360  # Convert to negative value if over 180°

        # For forward-only movement, set negative errors to zero
        if error < 0:
            error = 0
        return error

    def pid_control_motor(self, degrees_value, set_position, direction='reverse'):
        DEAD_ZONE_DEG_START = self.config['dead_zone_start']
        OFFSET = self.config['offset']
        slowdown_threshold = self.config['slowdown_threshold']
        max_control_change = self.config['max_control_change']
        max_degrees_per_second = self.config['max_degrees_per_second']

        current_time = time.time()
        delta_time = current_time - self.last_time
        self.last_time = current_time  # Update last_time here

        # Dead zone detection
        if degrees_value is None:
            if not self.in_dead_zone:
                # Entering dead zone
                self.in_dead_zone = True
                self.dead_zone_start_time = current_time

                # Ensure last_degrees_value and estimated_position are initialized properly
                if self.last_degrees_value is not None:
                    self.estimated_position = self.last_degrees_value
                elif set_position is not None:
                    self.estimated_position = set_position % 360
                else:
                    self.estimated_position = 0.0  # Default to 0 degrees if nothing is available
                print(f"{self.name}: Setting estimated_position from last_degrees_value: {self.estimated_position:.2f}°")

                # Calculate average speed before entering dead zone
                if self.speed_samples:
                    self.average_speed_before_dead_zone = sum(self.speed_samples) / len(self.speed_samples)
                else:
                    self.average_speed_before_dead_zone = self.last_valid_control_signal
                print(f"{self.name}: Entering dead zone. Maintaining average speed: {self.average_speed_before_dead_zone:.2f}%")

            else:
                # In dead zone, estimate position
                speed = self.average_speed_before_dead_zone  # Percentage of max speed
                if speed is None:
                    speed = self.config.get('default_speed', 10)  # Default speed if not available

                # Ensure that estimated_position is not None
                if self.estimated_position is None:
                    self.estimated_position = 0.0  # Default to 0 if uninitialized

                # Estimate degrees per second
                degrees_per_second = max_degrees_per_second * (speed / 100)

                # Ensure delta_time is valid and perform position estimation
                if delta_time > 0:
                    self.estimated_position = (self.estimated_position + degrees_per_second * delta_time) % 360
                degrees_value = self.estimated_position
                print(f"{self.name}: Estimated position in dead zone: {degrees_value:.2f}°")

        else:
            if self.in_dead_zone:
                # Exiting dead zone
                self.in_dead_zone = False
                self.dead_zone_start_time = None

                # Smooth transition by gradually adjusting the degrees_value
                transition_factor = 0.2  # Control the rate of transition (smaller values for slower transitions)
                degrees_value = (self.estimated_position * (1 - transition_factor)) + (degrees_value * transition_factor)

                print(f"{self.name}: Exiting dead zone. Smoothed adjusted position: {degrees_value:.2f}°")

            self.last_degrees_value = degrees_value

        # Ensure degrees_value is valid before proceeding
        if degrees_value is None:
            print(f"{self.name}: degrees_value is None, setting it to 0.0°")
            degrees_value = 0.0  # Default to 0 if degrees_value is None

        # Calculate error
        error = self.calculate_error(set_position, degrees_value)

        # Update set point in PID controller
        self.pid_controller.set_point = set_position

        # Compute control signal using PID controller
        control_signal = self.pid_controller.compute(degrees_value)

        # Apply rate limiter to control signal
        control_signal_change = control_signal - self.last_valid_control_signal
        if abs(control_signal_change) > max_control_change:
            control_signal = self.last_valid_control_signal + max_control_change * (1 if control_signal_change > 0 else -1)

        # Clamp control signal to 0-100%
        control_signal = max(0, min(100, control_signal))

        # Apply control signal based on error
        if error == 0:
            self.motor.set_speed(0)
            self.motor.stop()
            print(f"{self.name}: At or ahead of set position. Holding position.")
        elif error <= OFFSET:
            self.motor.set_speed(0)
            self.motor.stop()
            print(f"{self.name}: Motor stopped at target: {degrees_value:.2f} degrees")
        elif error <= slowdown_threshold:
            slowdown_factor = error / slowdown_threshold
            slow_control_signal = control_signal * slowdown_factor
            if direction == 'forward':
                self.motor.set_speed(slow_control_signal)
                self.motor.forward()
            else:
                self.motor.set_speed(slow_control_signal)
                self.motor.reverse()
            print(f"{self.name}: Slowing down: Potentiometer Value: {degrees_value:.2f}°, Error: {error:.2f}°, Control Signal: {slow_control_signal:.2f}%, Set Position: {set_position:.2f}°, Direction: {direction}")
        else:
            if direction == 'forward':
                self.motor.set_speed(control_signal)
                self.motor.forward()
            else:
                self.motor.set_speed(control_signal)
                self.motor.reverse()
            print(f"{self.name}: Moving {direction}: Potentiometer Value: {degrees_value:.2f}°, Error: {error:.2f}°, Control Signal: {control_signal:.2f}%, Set Position: {set_position:.2f}°")

        # Update last valid control signal
        self.last_valid_control_signal = control_signal

        # Keep sliding window of speed samples
        if not self.in_dead_zone:
            if len(self.speed_samples) >= self.config['num_samples_for_average']:
                self.speed_samples.pop(0)
            self.speed_samples.append(control_signal)

    def control_loop(self, stop_event, direction='reverse'):
        try:
            initial_pot_value = self.adc_reader.read_channel(self.channel)
            initial_angle = self.map_potentiometer_value_to_degrees(initial_pot_value)
            self.last_degrees_value = initial_angle
            self.sawtooth_generator.start_time = self.sawtooth_generator.current_millis()

            while not stop_event.is_set():
                pot_value = self.adc_reader.read_channel(self.channel)
                filtered_pot_value = self.spike_filter.filter(pot_value)
                degrees_value = None if filtered_pot_value is None else self.map_potentiometer_value_to_degrees(filtered_pot_value)
                self.sawtooth_generator.direction = direction  # Set the wave direction
                set_position = self.sawtooth_generator.get_set_position()
                self.pid_control_motor(degrees_value, set_position, direction)
                time.sleep(0.1)

        except Exception as e:
            print(f"{self.name}: Exception occurred: {e}")
        finally:
            self.motor.cleanup()
            print(f"{self.name}: Motor GPIO cleaned up")

