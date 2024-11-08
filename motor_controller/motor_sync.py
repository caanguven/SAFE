import RPi.GPIO as GPIO
import threading
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import math
import argparse

# ==========================
# ADCReader Class with Vector-Based Moving Average
# ==========================
class ADCReader:
    def __init__(self, spi_port=0, spi_device=0, num_samples=5):
        self.mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(spi_port, spi_device))
        self.lock = threading.Lock()
        self.num_samples = num_samples
        self.readings = {channel: [] for channel in range(8)}  # assuming 8 channels

    def read_channel(self, channel):
        with self.lock:
            adc_value = self.mcp.read_adc(channel)
            degrees = (adc_value / 1023.0) * 330.0  # Adjusted to 330 degrees
            # Convert degrees to radians for vector averaging
            radians = math.radians(degrees)
            self.readings[channel].append(radians)
            if len(self.readings[channel]) > self.num_samples:
                self.readings[channel].pop(0)
            # Compute average sine and cosine
            avg_sin = sum(math.sin(r) for r in self.readings[channel]) / len(self.readings[channel])
            avg_cos = sum(math.cos(r) for r in self.readings[channel]) / len(self.readings[channel])
            # Compute the averaged angle
            avg_radians = math.atan2(avg_sin, avg_cos)
            if avg_radians < 0:
                avg_radians += 2 * math.pi
            avg_degrees = math.degrees(avg_radians) % 330.0  # Keep within 0-330 degrees
            print(f"[ADCReader] Channel {channel} ADC Value: {adc_value}, Average: {avg_degrees:.2f} degrees")
            return avg_degrees

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
        duty_cycle = max(0, min(100, abs(duty_cycle)))
        self.pwm.ChangeDutyCycle(duty_cycle)
        print(f"[Motor] Set speed to {duty_cycle}% duty cycle")

    def forward(self):
        GPIO.output(self.in1_pin, GPIO.LOW)
        GPIO.output(self.in2_pin, GPIO.HIGH)
        print("[Motor] Moving forward")

    def reverse(self):
        GPIO.output(self.in1_pin, GPIO.HIGH)
        GPIO.output(self.in2_pin, GPIO.LOW)
        print("[Motor] Moving in reverse")

    def stop(self):
        GPIO.output(self.in1_pin, GPIO.LOW)
        GPIO.output(self.in2_pin, GPIO.LOW)
        self.set_speed(0)
        print("[Motor] Stopped")

    def cleanup(self):
        self.pwm.stop()
        print("[Motor] Cleaned up PWM")

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

        if error > 165.0:
            error -= 330.0
        elif error < -165.0:
            error += 330.0

        # Integral term with anti-windup
        self.integral += error * delta_time
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))

        # Derivative term
        derivative = (error - self.previous_error) / delta_time

        # Compute control signal
        control_signal = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # Limit control signal to prevent windup
        control_signal = max(-100, min(100, control_signal))

        print(f"[PIDController] Error: {error:.2f}, Integral: {self.integral:.2f}, Derivative: {derivative:.2f}, Control Signal: {control_signal:.2f}")

        self.previous_error = error
        self.last_time = current_time

        return control_signal

# ==========================
# SawtoothWaveGenerator Class
# ==========================
class SawtoothWaveGenerator:
    def __init__(self, period, amplitude, initial_angle, direction='forward', max_degrees=330.0):
        self.period = period  # in milliseconds
        self.amplitude = amplitude  # in degrees
        self.initial_angle = initial_angle
        self.direction = direction  # 'forward' or 'reverse'
        self.max_degrees = max_degrees
        self.start_time = self.current_millis()

    def current_millis(self):
        return int(time.time() * 1000)

    def get_set_position(self):
        t = self.current_millis() - self.start_time
        normalized_wave = (t % self.period) * (self.amplitude / self.period)

        if self.direction == 'forward':
            set_position = (normalized_wave + self.initial_angle) % self.max_degrees
        elif self.direction == 'reverse':
            set_position = (self.max_degrees - (normalized_wave + self.initial_angle)) % self.max_degrees
        else:
            raise ValueError("Invalid direction. Must be 'forward' or 'reverse'.")

        return set_position

    def set_direction(self, direction):
        if direction not in ['forward', 'reverse']:
            raise ValueError("Direction must be 'forward' or 'reverse'.")
        self.direction = direction

# ==========================
# MotorController Class
# ==========================
class MotorController:
    def __init__(self, motor, pid_controller, adc_reader, channel, sawtooth_generator, config, name='Motor'):
        self.motor = motor
        self.pid_controller = pid_controller
        self.adc_reader = adc_reader
        self.channel = channel
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
        degrees = value  # already mapped in ADCReader
        print(f"[{self.name}] Mapped ADC Average to {degrees:.2f} degrees")
        return degrees  # Keep within 0-330 degrees

    def calculate_error(self, set_position, current_angle):
        error = set_position - current_angle
        error = self.wrap_error(error)
        print(f"[{self.name}] Calculated Error: {error:.2f}° (Set Position: {set_position:.2f}°, Current Angle: {current_angle:.2f}°)")
        return error

    def wrap_error(self, error, max_range=330.0):
        # Wrap error to -max_range/2 to +max_range/2
        if error > max_range / 2:
            error -= max_range
        elif error < -max_range / 2:
            error += max_range
        return error

    def pid_control_motor(self, degrees_value, set_position):
        OFFSET = self.config['offset']
        slowdown_threshold = self.config['slowdown_threshold']
        max_control_change = self.config['max_control_change']
        MIN_CONTROL_SIGNAL = self.config['min_control_signal']

        error = self.calculate_error(set_position, degrees_value)
        self.pid_controller.set_point = set_position
        control_signal = self.pid_controller.compute(degrees_value)

        # Apply rate limiter to control signal
        control_signal_change = control_signal - self.last_valid_control_signal
        if abs(control_signal_change) > max_control_change:
            control_signal = self.last_valid_control_signal + max_control_change * (1 if control_signal_change > 0 else -1)

        # Clamp control signal
        control_signal = max(-100, min(100, control_signal))

        # Handle dead zone
        if abs(error) <= OFFSET:
            self.motor.set_speed(0)
            self.motor.stop()
            if not self.in_dead_zone:
                self.in_dead_zone = True
                print(f"[{self.name}] Entered dead zone: {degrees_value:.2f}° within offset {OFFSET}°")
        else:
            if self.in_dead_zone:
                # Exited dead zone
                self.in_dead_zone = False
                print(f"[{self.name}] Exited dead zone: {degrees_value:.2f}°")
            
            # Adjust control signal based on error magnitude
            if abs(error) <= slowdown_threshold:
                # Slow down as approaching target
                slowdown_factor = abs(error) / slowdown_threshold
                adjusted_control_signal = control_signal * slowdown_factor
                speed = max(abs(adjusted_control_signal), MIN_CONTROL_SIGNAL)
            else:
                # Full speed
                speed = max(abs(control_signal), MIN_CONTROL_SIGNAL)

            # Set motor direction and speed
            if control_signal > 0:
                self.motor.set_speed(speed)
                self.motor.forward()
                direction_text = 'forward'
            else:
                self.motor.set_speed(speed)
                self.motor.reverse()
                direction_text = 'reverse'

            print(f"[{self.name}] Moving {direction_text}: Potentiometer Value: {degrees_value:.2f}°, Error: {error:.2f}°, Control Signal: {speed:.2f}%, Set Position: {set_position:.2f}°")

        # Update last valid control signal
        self.last_valid_control_signal = control_signal

    def control_loop(self, stop_event, direction='forward'):
        try:
            initial_pot_value = self.adc_reader.read_channel(self.channel)
            initial_angle = self.map_potentiometer_value_to_degrees(initial_pot_value)
            self.last_degrees_value = initial_angle
            self.sawtooth_generator.start_time = self.sawtooth_generator.current_millis()

            while not stop_event.is_set():
                pot_value = self.adc_reader.read_channel(self.channel)
                degrees_value = self.map_potentiometer_value_to_degrees(pot_value)
                self.sawtooth_generator.direction = direction  # Set the wave direction
                set_position = self.sawtooth_generator.get_set_position()
                self.pid_control_motor(degrees_value, set_position)
                time.sleep(0.05)  # Control loop frequency (50ms)

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
        adc_reader = ADCReader(spi_port=0, spi_device=0, num_samples=5)

        # Shared configuration
        config = {
            'offset': 5,  # Degrees within which to stop the motor
            'slowdown_threshold': 20,  # Degrees within which to start slowing down
            'max_control_change': 10,  # Max change in control signal per loop
            'min_control_signal': 10  # Minimum control signal to move the motor
        }

        # PID constants (tune as necessary)
        Kp = 1.0
        Ki = 0.1
        Kd = 0.05

        # Define motors and their configurations
        motors_info = [
            {
                'name': 'Motor 1',
                'in1': 7,
                'in2': 26,
                'spd': 18,
                'adc_channel': 0,  # Potentiometer connected to ADC channel 0
            },
            {
                'name': 'Motor 3',
                'in1': 11,
                'in2': 32,
                'spd': 33,
                'adc_channel': 2,  # Potentiometer connected to ADC channel 2
            },
        ]

        # Create a list to hold motor controllers
        motor_controllers = []
        stop_event = threading.Event()

        # Create SawtoothWaveGenerator instances for each motor
        # Assuming all motors share the same wave generator for synchronization
        wave_generator = SawtoothWaveGenerator(
            period=5000,  # 5 seconds for a full cycle
            amplitude=330,  # Full rotation within 330 degrees
            initial_angle=0,
            direction=direction,  # 'forward' or 'reverse'
            max_degrees=330.0
        )

        # For each motor, create the necessary instances
        for motor_info in motors_info:
            # Create Motor instance
            motor = Motor(motor_info['in1'], motor_info['in2'], motor_info['spd'])

            # Create PIDController instance
            pid_controller = PIDController(Kp, Ki, Kd)

            # Read initial potentiometer value for initial angle
            initial_pot_value = adc_reader.read_channel(motor_info['adc_channel'])
            initial_angle = initial_pot_value  # Already mapped to degrees in ADCReader

            # Update SawtoothWaveGenerator's initial angle based on calibration
            # (Assuming calibration has been done and initial_angle is accurate)
            wave_generator.initial_angle = initial_angle
            wave_generator.start_time = wave_generator.current_millis()

            # Create MotorController instance
            motor_controller = MotorController(
                motor=motor,
                pid_controller=pid_controller,
                adc_reader=adc_reader,
                channel=motor_info['adc_channel'],
                sawtooth_generator=wave_generator,
                config=config,
                name=motor_info['name']
            )

            # Add to the list of motor controllers
            motor_controllers.append(motor_controller)

        # Create and start threads for each motor controller
        threads = []
        for mc in motor_controllers:
            t = threading.Thread(target=run_motor_controller, args=(mc, stop_event, direction))
            t.start()
            threads.append(t)

            print(f"[Main] {mc.name} Controller started. Target Position: 165°")

        print("[Main] Waiting for motors to reach initial positions...")
        # Assuming initial positioning is handled within the MotorController
        # Wait for some time to allow initial positioning
        time.sleep(2)  # Adjust as necessary based on motor speed

        print("[Main] Both motors have reached initial positions. Waiting for 5 seconds.")
        time.sleep(5)

        print("[Main] Starting synchronized movement.")

        # Reset wave generator to synchronize start
        wave_generator.start_time = wave_generator.current_millis()

        print("[Main] Press Ctrl+C to stop.")

        # Keep the main thread alive
        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[Main] Program stopped by user")
        stop_event.set()  # Signal all threads to stop
        for t in threads:
            t.join()  # Wait for all threads to finish
    except Exception as e:
        print(f"[Main] Exception occurred: {e}")
    finally:
        GPIO.cleanup()
        print("[Main] GPIO cleaned up")

if __name__ == "__main__":
    main()
