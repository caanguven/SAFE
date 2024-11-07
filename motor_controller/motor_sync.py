import RPi.GPIO as GPIO
import threading
import time
import argparse
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

# ==========================
# ADCReader Class
# ==========================
class ADCReader:
    def __init__(self, spi_port=0, spi_device=0):
        self.mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(spi_port, spi_device))
        self.lock = threading.Lock()

    def read_channel(self, channel):
        with self.lock:
            return self.mcp.read_adc(channel)

# ==========================
# Motor Class
# ==========================
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
        duty_cycle = max(0, min(100, abs(duty_cycle)))
        self.pwm.ChangeDutyCycle(duty_cycle)

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

# ==========================
# PIDController Class
# ==========================
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

        # Adjust error for minimal angle difference
        error = ((error + 180) % 360) - 180

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

# ==========================
# SawtoothWaveGenerator Class
# ==========================
class SawtoothWaveGenerator:
    def __init__(self, period, amplitude, initial_angle, direction='forward'):
        self.period = period  # in milliseconds
        self.amplitude = amplitude  # in degrees
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

# ==========================
# SpikeFilter Class
# ==========================
class SpikeFilter:
    def __init__(self, high_threshold, low_threshold):
        self.filter_active = False
        self.last_valid_reading = None
        self.high_threshold = high_threshold
        self.low_threshold = low_threshold

    def filter(self, new_value):
        if self.filter_active:
            # Exit filter when outside deadzone
            if new_value > self.high_threshold or new_value < self.low_threshold:
                print(f"Exiting dead zone: {new_value}")
                self.filter_active = False
                self.last_valid_reading = new_value
                return new_value
            else:
                print(f"Discarding invalid reading during dead zone: {new_value}")
                return None
        else:
            # Detect transition into deadzone
            if self.last_valid_reading is not None:
                if self.last_valid_reading > self.high_threshold and new_value < self.low_threshold:
                    print(f"Dead zone detected: last valid reading {self.last_valid_reading}, new reading {new_value}, starting filter")
                    self.filter_active = True
                    return None
            # Valid reading
            self.last_valid_reading = new_value
            return new_value

# ==========================
# MotorController Class
# ==========================
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

        # Initialization parameters
        self.initial_position = initial_position
        self.initialized = False

    def map_potentiometer_value_to_degrees(self, value):
        # Define dead zone ranges based on calibration
        if self.name == 'Motor 1':
            DEAD_ZONE_MIN = 150
            DEAD_ZONE_MAX = 200
            if 0 <= value < DEAD_ZONE_MIN:
                # Map ADC 0-DEAD_ZONE_MIN to degrees 0-90
                degrees = (value / DEAD_ZONE_MIN) * 90
            elif value > DEAD_ZONE_MAX and value <= 1023:
                # Map ADC DEAD_ZONE_MAX-1023 to degrees 90-360
                degrees = ((value - DEAD_ZONE_MAX) / (1023 - DEAD_ZONE_MAX)) * 270 + 90
            else:
                # Value is in dead zone
                return None
        elif self.name == 'Motor 3':
            DEAD_ZONE_MIN = 700
            DEAD_ZONE_MAX = 750
            if 0 <= value < DEAD_ZONE_MIN:
                # Map ADC 0-DEAD_ZONE_MIN to degrees 0-270
                degrees = (value / DEAD_ZONE_MIN) * 270
            elif value > DEAD_ZONE_MAX and value <= 1023:
                # Map ADC DEAD_ZONE_MAX-1023 to degrees 270-360
                degrees = ((value - DEAD_ZONE_MAX) / (1023 - DEAD_ZONE_MAX)) * 90 + 270
            else:
                # Value is in dead zone
                return None
        else:
            # Default linear mapping for other motors
            degrees = (value / 1023) * 360

        return degrees % 360

    def calculate_error(self, set_position, current_angle):
        # Calculate minimal angular difference
        error = (set_position - current_angle + 180) % 360 - 180
        return error

    def pid_control_motor(self, degrees_value, set_position, direction='forward', initialization=False):
        OFFSET = self.config['offset']
        slowdown_threshold = self.config['slowdown_threshold']
        max_control_change = self.config['max_control_change']
        MIN_CONTROL_SIGNAL = 10  # Minimum control signal to overcome static friction

        if degrees_value is None:
            print(f"{self.name}: degrees_value is None, cannot compute PID control.")
            self.motor.stop()
            return

        # Calculate error
        error = self.calculate_error(set_position, degrees_value)

        # Update set point in PID controller
        self.pid_controller.set_point = set_position

        # Compute control signal using PID controller
        control_signal = self.pid_controller.compute(degrees_value)

        # Apply rate limiter to control signal
        if not hasattr(self, 'last_control_signal'):
            self.last_control_signal = 0

        control_signal_change = control_signal - self.last_control_signal
        if abs(control_signal_change) > max_control_change:
            control_signal = self.last_control_signal + max_control_change * (1 if control_signal_change > 0 else -1)

        # Clamp control signal to -100 to 100
        control_signal = max(-100, min(100, control_signal))

        # During initialization, allow both forward and reverse
        if initialization:
            # Ensure a minimum control signal
            if abs(control_signal) < MIN_CONTROL_SIGNAL:
                control_signal = MIN_CONTROL_SIGNAL if control_signal > 0 else -MIN_CONTROL_SIGNAL

            # Apply control signal based on its sign
            if control_signal > 0:
                self.motor.set_speed(control_signal)
                self.motor.forward()
                dir_text = 'forward'
            else:
                self.motor.set_speed(abs(control_signal))
                self.motor.reverse()
                dir_text = 'reverse'

            print(f"{self.name}: [Init] Moving {dir_text}: Potentiometer Value: {degrees_value:.2f}°, Error: {error:.2f}°, Control Signal: {control_signal:.2f}%, Set Position: {set_position:.2f}°")
        else:
            # Normal operation
            if abs(error) <= OFFSET:
                self.motor.set_speed(0)
                self.motor.stop()
                print(f"{self.name}: Motor stopped at target: {degrees_value:.2f} degrees")
            elif abs(error) <= slowdown_threshold:
                # Slowing down
                slowdown_factor = abs(error) / slowdown_threshold
                slow_control_signal = control_signal * slowdown_factor
                speed = max(abs(slow_control_signal), MIN_CONTROL_SIGNAL)  # Ensure minimum speed

                if slow_control_signal > 0:
                    self.motor.set_speed(speed)
                    self.motor.forward()
                    dir_text = 'forward'
                else:
                    self.motor.set_speed(speed)
                    self.motor.reverse()
                    dir_text = 'reverse'

                print(f"{self.name}: Slowing down ({dir_text}): Potentiometer Value: {degrees_value:.2f}°, Error: {error:.2f}°, Control Signal: {control_signal:.2f}%, Set Position: {set_position:.2f}°")
            else:
                # Moving normally
                speed = max(abs(control_signal), MIN_CONTROL_SIGNAL)  # Ensure minimum speed

                if control_signal > 0:
                    self.motor.set_speed(speed)
                    self.motor.forward()
                    dir_text = 'forward'
                else:
                    self.motor.set_speed(speed)
                    self.motor.reverse()
                    dir_text = 'reverse'

                print(f"{self.name}: Moving {dir_text}: Potentiometer Value: {degrees_value:.2f}°, Error: {error:.2f}°, Control Signal: {control_signal:.2f}%, Set Position: {set_position:.2f}°")

        # Update last control signal
        self.last_control_signal = control_signal

    def control_loop(self, stop_event, direction='forward'):
        try:
            # Initial positioning phase
            if not self.initialized:
                print(f"{self.name}: Moving to initial position {self.initial_position}°")
                while not stop_event.is_set():
                    pot_value = self.adc_reader.read_channel(self.channel)
                    print(f"{self.name}: Raw ADC Value: {pot_value}")  # Debugging
                    filtered_pot_value = self.spike_filter.filter(pot_value)
                    degrees_value = None if filtered_pot_value is None else self.map_potentiometer_value_to_degrees(filtered_pot_value)
                    print(f"{self.name}: Mapped Degrees Value: {degrees_value}")  # Debugging

                    set_position = self.initial_position

                    if degrees_value is not None:
                        error = self.calculate_error(set_position, degrees_value)
                        if abs(error) <= self.config['offset']:
                            self.motor.stop()
                            print(f"{self.name}: Reached initial position {self.initial_position}°")
                            time.sleep(1)  # Brief pause
                            self.initialized = True
                            self.sawtooth_generator.start_time = self.sawtooth_generator.current_millis()  # Reset start time for sawtooth wave
                            break
                        else:
                            # Determine direction based on error
                            init_direction = 'forward' if error > 0 else 'reverse'
                            self.pid_control_motor(degrees_value, set_position, direction=init_direction, initialization=True)
                    else:
                        # degrees_value is None
                        # Check if set_position is initial_position and within deadzone
                        if self.name == 'Motor 3' and set_position == 270:
                            self.motor.stop()
                            print(f"{self.name}: Reached initial position {self.initial_position}° within deadzone.")
                            self.initialized = True
                            self.sawtooth_generator.start_time = self.sawtooth_generator.current_millis()  # Reset start time for sawtooth wave
                            break
                        else:
                            print(f"{self.name}: degrees_value is None during initialization, set_position not reached.")

                    time.sleep(0.1)

            # Normal control loop
            while not stop_event.is_set():
                pot_value = self.adc_reader.read_channel(self.channel)
                print(f"{self.name}: Raw ADC Value: {pot_value}")  # Debugging
                filtered_pot_value = self.spike_filter.filter(pot_value)
                degrees_value = None if filtered_pot_value is None else self.map_potentiometer_value_to_degrees(filtered_pot_value)
                print(f"{self.name}: Mapped Degrees Value: {degrees_value}")  # Debugging

                if self.initialized:
                    set_position = self.sawtooth_generator.get_set_position()
                    print(f"{self.name}: Set Position: {set_position:.2f}°")  # Debugging
                    # During normal operation, restrict direction to forward
                    self.pid_control_motor(degrees_value, set_position, direction='forward', initialization=False)
                else:
                    # Should not reach here, but just in case
                    print(f"{self.name}: Not initialized yet.")

                time.sleep(0.1)

        except Exception as e:
            print(f"{self.name}: Exception occurred: {e}")
        finally:
            self.motor.cleanup()
            print(f"{self.name}: Motor GPIO cleaned up")

# ==========================
# Main Function
# ==========================
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
            'offset': 5,  # Degrees within which to stop the motor
            'slowdown_threshold': 20,  # Degrees within which to start slowing down
            'max_control_change': 5,  # Max change in control signal per loop
        }

        # PID constants (tune as necessary)
        Kp = 0.5   # Reduced for smoother control
        Ki = 0.05  # Reduced to prevent integral windup
        Kd = 0.15  # Adjusted for better derivative response

        # Define motors and their configurations
        motors_info = [
            {
                'id': 1,
                'name': 'Motor 1',
                'in1': 7,
                'in2': 26,
                'spd': 18,
                'adc_channel': 0,
                'initial_position': 90
            },
            {
                'id': 3,
                'name': 'Motor 3',
                'in1': 11,
                'in2': 32,
                'spd': 33,
                'adc_channel': 2,
                'initial_position': 270
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

            # Create SpikeFilter instance with appropriate thresholds
            if motor_info['name'] == 'Motor 1':
                # For Motor 1, high_threshold=300, low_threshold=30
                spike_filter = SpikeFilter(high_threshold=300, low_threshold=30)
            elif motor_info['name'] == 'Motor 3':
                # For Motor 3, high_threshold=750, low_threshold=700
                spike_filter = SpikeFilter(high_threshold=750, low_threshold=700)
            else:
                # Default thresholds
                spike_filter = SpikeFilter(high_threshold=500, low_threshold=500)

            # Initialize SawtoothWaveGenerator
            sawtooth_generator = SawtoothWaveGenerator(
                period=4000,  # 4 seconds for a full cycle
                amplitude=360,
                initial_angle=0,
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
                initial_position=motor_info['initial_position']  # Pass initial position
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
        print("\nProgram stopped by user")
        stop_event.set()  # Signal all threads to stop
        for t in threads:
            t.join()  # Wait for all threads to finish
    finally:
        GPIO.cleanup()
        print("GPIO cleaned up")

if __name__ == "__main__":
    main()
