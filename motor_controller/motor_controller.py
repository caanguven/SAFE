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

    def pid_control_motor(self, degrees_value, set_position):
        # Configuration parameters
        DEAD_ZONE_DEG_START = self.config['dead_zone_start']
        OFFSET = self.config['offset']
        NUM_SAMPLES_FOR_AVERAGE = self.config['num_samples_for_average']
        slowdown_threshold = self.config['slowdown_threshold']

        current_time = time.time()
        delta_time = current_time - self.last_time
        self.last_time = current_time  # Update last_time here

        # Dead zone detection
        if degrees_value is None:
            if not self.in_dead_zone:
                # Entering dead zone
                self.in_dead_zone = True
                self.dead_zone_start_time = current_time

                # Ensure last_degrees_value is not None
                if self.last_degrees_value is not None:
                    self.estimated_position = self.last_degrees_value
                else:
                    # Set to set_position or a default value
                    self.estimated_position = set_position % 360
                    print(f"{self.name}: last_degrees_value is None, setting estimated_position to set_position ({self.estimated_position:.2f}°)")

                if self.speed_samples:
                    self.average_speed_before_dead_zone = sum(self.speed_samples) / len(self.speed_samples)
                else:
                    self.average_speed_before_dead_zone = self.last_valid_control_signal
                print(f"{self.name}: Entering dead zone. Maintaining average speed: {self.average_speed_before_dead_zone:.2f}%")
            else:
                # In dead zone, estimate position
                time_in_dead_zone = current_time - self.dead_zone_start_time
                speed = self.average_speed_before_dead_zone  # Percentage of max speed
                # Estimate position based on speed and delta_time
                degrees_per_second = self.config['max_degrees_per_second'] * (speed / 100)
                self.estimated_position = (self.estimated_position + degrees_per_second * delta_time) % 360
                degrees_value = self.estimated_position
                print(f"{self.name}: Estimated position in dead zone: {degrees_value:.2f}°")
        else:
            if self.in_dead_zone:
                # Exiting dead zone
                self.in_dead_zone = False
                self.dead_zone_start_time = None
                # Smooth transition by using estimated position
                degrees_value = (self.estimated_position + degrees_value) / 2  # Average estimated and actual
                print(f"{self.name}: Exiting dead zone. Adjusted position: {degrees_value:.2f}°")
            self.last_degrees_value = degrees_value

        # Calculate error
        error = self.calculate_error(set_position, degrees_value)

        # Update set point in PID controller
        self.pid_controller.set_point = set_position

        # Compute control signal using PID controller
        control_signal = self.pid_controller.compute(degrees_value)

        # Apply rate limiter to control signal
        max_control_change = self.config['max_control_change']
        control_signal_change = control_signal - self.last_valid_control_signal
        if abs(control_signal_change) > max_control_change:
            control_signal = self.last_valid_control_signal + max_control_change * (1 if control_signal_change > 0 else -1)

        # Clamp control signal to 0-100%
        control_signal = max(0, min(100, control_signal))

        # Apply control signal based on error
        if error == 0:
            # Motor is at or ahead of set position; stop or maintain current speed
            self.motor.set_speed(0)
            self.motor.stop()
            print(f"{self.name}: At or ahead of set position. Holding position.")
        elif error <= OFFSET:
            # Stop the motor if within target range
            self.motor.set_speed(0)
            self.motor.stop()
            print(f"{self.name}: Motor stopped at target: {degrees_value:.2f} degrees")
        elif error <= slowdown_threshold:
            # Reduce speed as approaching target
            slowdown_factor = error / slowdown_threshold
            slow_control_signal = control_signal * slowdown_factor
            self.motor.set_speed(slow_control_signal)
            self.motor.forward()
            print(f"{self.name}: Slowing down: Potentiometer Value: {degrees_value:.2f}°, Error: {error:.2f}°, Control Signal: {slow_control_signal:.2f}%, Set Position: {set_position:.2f}°")
        else:
            # Move forward at calculated speed
            self.motor.set_speed(control_signal)
            self.motor.forward()
            print(f"{self.name}: Moving forward: Potentiometer Value: {degrees_value:.2f}°, Error: {error:.2f}°, Control Signal: {control_signal:.2f}%, Set Position: {set_position:.2f}°")

        # Update last valid control signal
        self.last_valid_control_signal = control_signal

        # Keep sliding window of speed samples
        if not self.in_dead_zone:
            if len(self.speed_samples) >= NUM_SAMPLES_FOR_AVERAGE:
                self.speed_samples.pop(0)
            self.speed_samples.append(control_signal)

    def control_loop(self, stop_event):
        try:
            # Removed self.last_time initialization here
            # Read initial potentiometer value
            initial_pot_value = self.adc_reader.read_channel(self.channel)
            initial_angle = self.map_potentiometer_value_to_degrees(initial_pot_value)
            self.last_degrees_value = initial_angle
            print(f"{self.name}: Initial motor angle set to: {initial_angle:.2f} degrees")

            # Initialize sawtooth generator start time
            self.sawtooth_generator.start_time = self.sawtooth_generator.current_millis()

            while not stop_event.is_set():
                # Read ADC value
                pot_value = self.adc_reader.read_channel(self.channel)

                # Apply spike filter
                filtered_pot_value = self.spike_filter.filter(pot_value)

                if filtered_pot_value is None:
                    degrees_value = None  # No valid reading
                else:
                    degrees_value = self.map_potentiometer_value_to_degrees(filtered_pot_value)
                    self.last_degrees_value = degrees_value

                # Get set position from sawtooth wave generator
                set_position = self.sawtooth_generator.get_set_position()

                # Control motor
                self.pid_control_motor(degrees_value, set_position)

                time.sleep(0.1)

        except Exception as e:
            print(f"{self.name}: Exception occurred: {e}")
        finally:
            self.motor.cleanup()
            print(f"{self.name}: Motor GPIO cleaned up")
