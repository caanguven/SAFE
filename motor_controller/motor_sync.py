import RPi.GPIO as GPIO
import threading
import time
import argparse
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

# ADCReader Class
class ADCReader:
    def __init__(self, spi_port=0, spi_device=0):
        self.mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(spi_port, spi_device))
        self.lock = threading.Lock()

    def read_channel(self, channel):
        with self.lock:
            return self.mcp.read_adc(channel)

# Motor Class
class Motor:
    def __init__(self, in1_pin, in2_pin, spd_pin, pwm_frequency=1000):
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.spd_pin = spd_pin
        self.pwm_frequency = pwm_frequency

        # Setup GPIO pins
        GPIO.setup(self.in1_pin, GPIO.OUT)
        GPIO.setup(self.in2_pin, GPIO.OUT)
        GPIO.setup(self.spd_pin, GPIO.OUT)

        # Setup PWM
        self.pwm = GPIO.PWM(self.spd_pin, self.pwm_frequency)
        self.pwm.start(0)  # Start with 0% duty cycle (motor off)

    def set_speed(self, duty_cycle):
        # Clamp duty_cycle to 0-100%
        duty_cycle = max(-100, min(100, duty_cycle))
        self.pwm.ChangeDutyCycle(abs(duty_cycle))

    def forward(self):
        GPIO.output(self.in1_pin, GPIO.LOW)
        GPIO.output(self.in2_pin, GPIO.HIGH)

    def reverse(self):
        GPIO.output(self.in1_pin, GPIO.HIGH)
        GPIO.output(self.in2_pin, GPIO.LOW)

    def stop(self):
        GPIO.output(self.in1_pin, GPIO.LOW)
        GPIO.output(self.in2_pin, GPIO.LOW)
        self.set_speed(0)

    def cleanup(self):
        self.pwm.stop()

# PIDController Class
class PIDController:
    def __init__(self, Kp, Ki, Kd, set_point=0, integral_limit=1000):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.set_point = set_point

        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()
        self.integral_limit = integral_limit  # Max absolute value for integral term

    def compute(self, current_value):
        current_time = time.time()
        delta_time = current_time - self.last_time
        if delta_time <= 0.0:
            delta_time = 0.0001  # Avoid division by zero

        # Calculate error considering wrap-around
        error = self.set_point - current_value

        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        # Integral term with anti-windup
        self.integral += error * delta_time
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))

        # Derivative term
        derivative = (error - self.previous_error) / delta_time

        # Compute control signal
        control_signal = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # Update state
        self.previous_error = error
        self.last_time = current_time

        return control_signal

# SawtoothWaveGenerator Class
class SawtoothWaveGenerator:
    def __init__(self, period, amplitude, initial_angle, direction='forward'):
        self.period = period
        self.amplitude = amplitude
        self.initial_angle = initial_angle
        self.direction = direction  # 'forward' or 'reverse'
        self.start_time = self.current_millis()

    def current_millis(self):
        return int(time.time() * 1000)

    def get_set_position(self):
        t = self.current_millis() - self.start_time
        normalized_wave = (t % self.period) * (self.amplitude / self.period)

        if self.direction == 'forward':
            set_position = (normalized_wave + self.initial_angle) % 360
        elif self.direction == 'reverse':
            set_position = (360 - (normalized_wave + self.initial_angle)) % 360
        else:
            raise ValueError("Invalid direction. Must be 'forward' or 'reverse'.")

        return set_position

    def set_direction(self, direction):
        if direction not in ['forward', 'reverse']:
            raise ValueError("Direction must be 'forward' or 'reverse'.")
        self.direction = direction

# SpikeFilter Class
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

# MotorController Class
class MotorController:
    def __init__(self, motor, pid_controller, adc_reader, channel, spike_filter, sawtooth_generator, config, name='Motor', initial_position=0):
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

        # Initialization parameters
        self.initial_position = initial_position
        self.initialized = False

    def map_potentiometer_value_to_degrees(self, value):
        # Adjust mapping to account for dead zone
        if 0 <= value <= 150:
            # Map ADC 0-150 to degrees 0-114
            degrees = value * (114 / 150)
        elif 700 <= value <= 1023:
            # Map ADC 700-1023 to degrees 114-360
            degrees = (114 + (value - 700) * ((360 - 114) / (1023 - 700)))
        else:
            # Value is in dead zone
            return None
        return degrees % 360

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

    def pid_control_motor(self, degrees_value, set_position, direction='forward'):
        DEAD_ZONE_DEG_START = self.config['dead_zone_start']
        OFFSET = self.config['offset']
        slowdown_threshold = self.config['slowdown_threshold']
        max_control_change = self.config['max_control_change']

        current_time = time.time()
        delta_time = current_time - self.last_time
        self.last_time = current_time  # Update last_time here

        if degrees_value is None:
            print(f"{self.name}: degrees_value is None, cannot compute PID control.")
            self.motor.stop()
            return

        # Calculate error based on direction (forward or reverse)
        error = self.calculate_error(set_position, degrees_value, direction)

        # Update set point in PID controller
        self.pid_controller.set_point = set_position

        # Compute control signal using PID controller
        control_signal = self.pid_controller.compute(degrees_value)

        # Apply rate limiter to control signal
        control_signal_change = control_signal - self.last_valid_control_signal
        if abs(control_signal_change) > max_control_change:
            control_signal = self.last_valid_control_signal + max_control_change * (1 if control_signal_change > 0 else -1)

        # Clamp control signal to -100 to 100
        control_signal = max(-100, min(100, control_signal))

        # Apply control signal based on error
        if abs(error) == 0:
            self.motor.set_speed(0)
            self.motor.stop()
            print(f"{self.name}: At or ahead of set position. Holding position.")
        elif abs(error) <= OFFSET:
            self.motor.set_speed(0)
            self.motor.stop()
            print(f"{self.name}: Motor stopped at target: {degrees_value:.2f} degrees")
        elif abs(error) <= slowdown_threshold:
            slowdown_factor = abs(error) / slowdown_threshold
            slow_control_signal = control_signal * slowdown_factor
            speed = abs(slow_control_signal)

            if slow_control_signal > 0:
                self.motor.set_speed(speed)
                self.motor.forward()
                dir_text = 'forward'
            else:
                self.motor.set_speed(speed)
                self.motor.reverse()
                dir_text = 'reverse'

            print(f"{self.name}: Slowing down ({dir_text}): Potentiometer Value: {degrees_value:.2f}°, Error: {error:.2f}°, Control Signal: {speed:.2f}%, Set Position: {set_position:.2f}°")
        else:
            speed = abs(control_signal)
            if control_signal > 0:
                self.motor.set_speed(speed)
                self.motor.forward()
                dir_text = 'forward'
            else:
                self.motor.set_speed(speed)
                self.motor.reverse()
                dir_text = 'reverse'

            print(f"{self.name}: Moving {dir_text}: Potentiometer Value: {degrees_value:.2f}°, Error: {error:.2f}°, Control Signal: {speed:.2f}%, Set Position: {set_position:.2f}°")

        # Update last valid control signal
        self.last_valid_control_signal = control_signal

        # Keep sliding window of speed samples
        if not self.in_dead_zone:
            if len(self.speed_samples) >= self.config['num_samples_for_average']:
                self.speed_samples.pop(0)
            self.speed_samples.append(abs(control_signal))

    def control_loop(self, stop_event, direction='reverse'):
        try:
            # Initial positioning phase
            if not self.initialized:
                print(f"{self.name}: Moving to initial position {self.initial_position}°")
                while not stop_event.is_set():
                    pot_value = self.adc_reader.read_channel(self.channel)
                    filtered_pot_value = self.spike_filter.filter(pot_value)
                    degrees_value = None if filtered_pot_value is None else self.map_potentiometer_value_to_degrees(filtered_pot_value)

                    if degrees_value is not None:
                        error = self.calculate_error(self.initial_position, degrees_value, direction)
                        if abs(error) <= self.config['offset']:
                            self.motor.stop()
                            print(f"{self.name}: Reached initial position {self.initial_position}°")
                            time.sleep(3)  # Pause for 3 seconds
                            self.initialized = True
                            self.sawtooth_generator.start_time = self.sawtooth_generator.current_millis()  # Reset start time for sawtooth wave
                            break
                        else:
                            self.pid_control_motor(degrees_value, self.initial_position, direction)
                    else:
                        print(f"{self.name}: degrees_value is None during initialization.")

                    time.sleep(0.1)

            # Normal control loop
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

# Main Function
def run_motor_controller(motor_controller, stop_event, direction):
    motor_controller.control_loop(stop_event, direction)

def main():
    try:
        # Set up argument parser
        parser = argparse.ArgumentParser(description='Motor Control Program')
        parser.add_argument('direction', choices=['forward', 'reverse'], help='Direction to move the motor')
        args = parser.parse_args()

        # Extract the direction
        direction = args.direction

        # Set up GPIO
        GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering

        # Create ADCReader instance
        adc_reader = ADCReader(spi_port=0, spi_device=0)

        # Shared configuration
        config = {
            'dead_zone_start': 330,
            'offset': 5,
            'num_samples_for_average': 5,
            'slowdown_threshold': 20,
            'max_control_change': 5,
            'max_degrees_per_second': 60
        }

        # PID constants (tune as necessary)
        Kp = 0.1
        Ki = 0.01
        Kd = 0.1

        # Define motors and their configurations
        motors_info = [
            {
                'id': 1,
                'name': 'Motor 1',
                'in1': 7,
                'in2': 26,
                'spd': 18,
                'adc_channel': 0,
            },
            {
                'id': 3,
                'name': 'Motor 3',
                'in1': 11,
                'in2': 32,
                'spd': 33,
                'adc_channel': 2,
            },
        ]

        # Create a list to hold motor controllers
        motor_controllers = []
        stop_event = threading.Event()

        # For each motor, create the necessary instances
        for motor_info in motors_info:
            # Create Motor instance
            motor = Motor(motor_info['in1'], motor_info['in2'], motor_info['spd'])

            # Create PIDController instance
            pid_controller = PIDController(Kp, Ki, Kd)

            # Create SpikeFilter instance
            spike_filter = SpikeFilter()

            # Read initial potentiometer value for initial angle
            initial_pot_value = adc_reader.read_channel(motor_info['adc_channel'])
            initial_angle = 0  # Start from 0 for sawtooth generator

            # Determine initial position based on motor id
            if motor_info['id'] == 1:
                initial_position = 90
            elif motor_info['id'] == 3:
                initial_position = 270
            else:
                initial_position = 0  # default or adjust as needed

            # Create SawtoothWaveGenerator instance with direction
            sawtooth_generator = SawtoothWaveGenerator(
                period=4000,
                amplitude=360,
                initial_angle=initial_angle,
                direction=direction  # Pass the direction here
            )

            # Create MotorController instance
            motor_controller = MotorController(
                motor=motor,
                pid_controller=pid_controller,
                adc_reader=adc_reader,
                channel=motor_info['adc_channel'],
                spike_filter=spike_filter,
                sawtooth_generator=sawtooth_generator,
                config=config,
                name=motor_info['name'],  # Pass the name for logging
                initial_position=initial_position  # Pass initial position
            )

            # Add to the list of motor controllers
            motor_controllers.append(motor_controller)

        # Create and start threads for each motor controller
        threads = []
        for mc in motor_controllers:
            t = threading.Thread(target=run_motor_controller, args=(mc, stop_event, direction))
            t.start()
            threads.append(t)

        # Wait for KeyboardInterrupt to stop
        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Program stopped by user")
        stop_event.set()  # Signal all threads to stop
        for t in threads:
            t.join()  # Wait for all threads to finish
    finally:
        GPIO.cleanup()
        print("GPIO cleaned up")

if __name__ == "__main__":
    main()
