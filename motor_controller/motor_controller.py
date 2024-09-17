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

    def calculate_error(self, set_position, current_angle, direction='reverse'):
        if direction == 'reverse':
            # Reverse logic: calculate error for reverse motion
            error = (current_angle - set_position + 360) % 360
            if error > 180:
                error = error - 360
        else:
            # Default forward error calculation
            error = (set_position - current_angle + 360) % 360
            if error > 180:
                error = error - 360

        # Do not clamp errors to zero for reverse; they should remain negative or positive
        return error



    def pid_control_motor(self, degrees_value, set_position, direction):
        if degrees_value is None:
            return
    
        # Calculate the error directly using degrees
        error = self.calculate_circular_error(set_position, degrees_value)
    
        # Get the current time
        current_time = time.time()
        delta_time = current_time - self.last_time
    
        if delta_time >= 0.01:  # Ensure the loop doesn't update too frequently
            # Calculate integral (sum of errors over time)
            self.integral += error * delta_time
    
            # Calculate derivative (rate of change of error)
            derivative = (error - self.previous_error) / delta_time
    
            # Calculate the control signal using the PID controller
            control_signal = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
    
            # Clamp control signal to valid PWM range (0-100%)
            control_signal = max(0, min(100, control_signal))  # Clamping to 0-100
    
            # Slow down gradually as the error approaches the target
            slowdown_threshold = 20  # Degrees within which to slow down the motor
    
            if abs(error) <= self.OFFSET:
                # Stop the motor if within the target range (small error)
                self.motor.stop()
                print(f"{self.name}: Motor stopped at target: {degrees_value:.2f} degrees ({direction})")
            elif abs(error) <= slowdown_threshold:
                # Reduce speed as we approach the target
                slowdown_factor = abs(error) / slowdown_threshold  # Proportional slowdown
                slow_control_signal = control_signal * slowdown_factor
                self.motor.set_speed(slow_control_signal)
                self.motor.reverse() if direction == 'reverse' else self.motor.forward()
                print(f"{self.name}: Slowing down ({direction}): Potentiometer Value: {degrees_value:.2f}°, Error: {error:.2f}°, Control Signal: {slow_control_signal:.2f}%, Set Position: {set_position:.2f}°")
            else:
                # Always move forward (ignore backward)
                self.motor.set_speed(control_signal)
                self.motor.reverse() if direction == 'reverse' else self.motor.forward()
                print(f"{self.name}: Moving {direction}: Potentiometer Value: {degrees_value:.2f}°, Error: {error:.2f}°, Control Signal: {control_signal:.2f}%, Set Position: {set_position:.2f}°")
    
            # Update previous error, time, and store the last valid control signal
            self.previous_error = error
            self.last_time = current_time
            self.last_valid_control_signal = control_signal  # Store the last valid signal
    
            # Keep a sliding window of control signal samples to calculate average speed
            if not self.in_dead_zone:
                if len(self.speed_samples) >= self.config['num_samples_for_average']:
                    self.speed_samples.pop(0)  # Remove the oldest sample
                self.speed_samples.append(control_signal)  # Add the new control signal to the list

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

