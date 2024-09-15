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

    def map_potentiometer_value_to_degrees(self, value):
        # Potentiometer values range from 0 to 1023
        # Map to 0 to 330 degrees (since the potentiometer only reads up to 330°)
        return value * (330 / 1023)

    def calculate_error(self, set_position, current_angle):
        # Calculate the error, handling wrap-around
        error = set_position - current_angle

        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        return error

    def pid_control_motor(self, degrees_value, set_position):
        # Configuration parameters
        DEAD_ZONE_DEG_START = self.config['dead_zone_start']
        OFFSET = self.config['offset']
        NUM_SAMPLES_FOR_AVERAGE = self.config['num_samples_for_average']
        slowdown_threshold = self.config['slowdown_threshold']

        # Dead zone detection
        if degrees_value >= DEAD_ZONE_DEG_START:
            if not self.in_dead_zone:
                # Entering dead zone, calculate average speed
                self.in_dead_zone = True
                if self.speed_samples:
                    self.average_speed_before_dead_zone = sum(self.speed_samples) / len(self.speed_samples)
                else:
                    self.average_speed_before_dead_zone = self.last_valid_control_signal
                print(f"{self.name}: Entering dead zone. Maintaining average speed: {self.average_speed_before_dead_zone:.2f}%")

            # Continue applying the average speed
            self.motor.set_speed(self.average_speed_before_dead_zone)
            self.motor.forward()
            print(f"{self.name}: Moving in dead zone. Control signal: {self.average_speed_before_dead_zone:.2f}%")
            return

        # Exit dead zone when degrees go back below DEAD_ZONE_DEG_START
        if self.in_dead_zone and degrees_value < DEAD_ZONE_DEG_START:
            self.in_dead_zone = False
            print(f"{self.name}: Exiting dead zone. Resuming normal control.")

        # Calculate error
        error = self.calculate_error(set_position, degrees_value)

        # Update set point in PID controller
        self.pid_controller.set_point = set_position

        # Compute control signal using PID controller
        control_signal = self.pid_controller.compute(degrees_value)

        # Clamp control signal to -100% to 100%
        control_signal = max(-100, min(100, control_signal))

        # Determine motor direction and speed
        if abs(error) <= OFFSET:
            # Stop the motor if within target range
            self.motor.stop()
            print(f"{self.name}: Motor stopped at target: {degrees_value:.2f} degrees")
        else:
            speed = abs(control_signal)
            # Ensure speed is within 0-100%
            speed = max(0, min(100, speed))

            if control_signal > 0:
                self.motor.forward()
                direction = 'forward'
            else:
                self.motor.backward()
                direction = 'backward'

            self.motor.set_speed(speed)
            print(f"{self.name}: Moving {direction}: Potentiometer Value: {degrees_value:.2f}, Error: {error:.2f}, Control Signal: {control_signal:.2f}%, Set Position: {set_position:.2f}")

        # Update last valid control signal
        self.last_valid_control_signal = abs(control_signal)

        # Keep sliding window of speed samples
        if len(self.speed_samples) >= NUM_SAMPLES_FOR_AVERAGE:
            self.speed_samples.pop(0)
        self.speed_samples.append(abs(control_signal))

    def control_loop(self, stop_event):
        try:
            # Read initial potentiometer value
            initial_pot_value = self.adc_reader.read_channel(self.channel)
            initial_angle = self.map_potentiometer_value_to_degrees(initial_pot_value)
            print(f"{self.name}: Initial motor angle set to: {initial_angle:.2f} degrees")

            # Initialize sawtooth generator start time
            self.sawtooth_generator.start_time = self.sawtooth_generator.current_millis()

            while not stop_event.is_set():
                # Read ADC value
                pot_value = self.adc_reader.read_channel(self.channel)

                # Apply spike filter
                filtered_pot_value = self.spike_filter.filter(pot_value)

                if filtered_pot_value is None:
                    continue

                degrees_value = self.map_potentiometer_value_to_degrees(filtered_pot_value)

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
