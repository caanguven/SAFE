import RPi.GPIO as GPIO
import threading
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import math

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
        self.integral_limit = integral_limit

    def compute(self, current_value):
        current_time = time.time()
        delta_time = current_time - self.last_time
        if delta_time <= 0.0:
            delta_time = 0.0001

        error = self.set_point - current_value
        error = self.wrap_error(error)

        self.integral += error * delta_time
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))

        derivative = (error - self.previous_error) / delta_time
        control_signal = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # Limit control signal to prevent windup
        control_signal = max(-100, min(100, control_signal))

        print(f"[PIDController] Error: {error:.2f}, Integral: {self.integral:.2f}, Derivative: {derivative:.2f}, Control Signal: {control_signal:.2f}")

        self.previous_error = error
        self.last_time = current_time

        return control_signal

    @staticmethod
    def wrap_error(error, max_range=330.0):
        # Wrap error to -max_range/2 to +max_range/2
        if error > max_range / 2:
            error -= max_range
        elif error < -max_range / 2:
            error += max_range
        return error

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
    def __init__(self, motor, pid_controller, adc_reader, channel, config,
                 name='Motor', target_position=0,
                 wave_generator=None, event_reached_position=None, event_start_synchronized_movement=None):
        self.motor = motor
        self.pid_controller = pid_controller
        self.adc_reader = adc_reader
        self.channel = channel
        self.config = config
        self.name = name
        self.target_position = target_position
        self.wave_generator = wave_generator
        self.event_reached_position = event_reached_position
        self.event_start_synchronized_movement = event_start_synchronized_movement
        self.state = 'initializing'
        self.at_target_position = False
        self.last_control_signal = 0

    def map_potentiometer_value_to_degrees(self, value):
        degrees = value  # already mapped in ADCReader
        print(f"[{self.name}] Mapped ADC Average to {degrees:.2f} degrees")
        return degrees  # Keep within 0-330 degrees

    def calculate_error(self, set_position, current_angle):
        error = set_position - current_angle
        error = self.pid_controller.wrap_error(error)
        print(f"[{self.name}] Calculated Error: {error:.2f}° (Set Position: {set_position}°, Current Angle: {current_angle:.2f}°)")
        return error

    def pid_control_motor(self, degrees_value, set_position):
        OFFSET = self.config['offset']
        max_control_change = self.config['max_control_change']
        MIN_CONTROL_SIGNAL = self.config['min_control_signal']

        error = self.calculate_error(set_position, degrees_value)
        self.pid_controller.set_point = set_position
        control_signal = self.pid_controller.compute(degrees_value)

        # Limit control signal change
        control_signal_change = control_signal - self.last_control_signal
        if abs(control_signal_change) > max_control_change:
            control_signal = self.last_control_signal + max_control_change * (1 if control_signal_change > 0 else -1)

        # Clamp control signal
        control_signal = max(-100, min(100, control_signal))

        if abs(error) <= OFFSET:
            self.motor.set_speed(0)
            self.motor.stop()
            if not self.at_target_position:
                self.at_target_position = True
                if self.event_reached_position:
                    self.event_reached_position.set()
                print(f"[{self.name}] Reached target position within offset: {degrees_value:.2f}°")
        else:
            speed = max(abs(control_signal), MIN_CONTROL_SIGNAL)
            if control_signal > 0:
                self.motor.set_speed(speed)
                self.motor.forward()
            else:
                self.motor.set_speed(speed)
                self.motor.reverse()

            print(f"[{self.name}] Moving motor: Potentiometer Value: {degrees_value:.2f}°, Error: {error:.2f}°, Control Signal: {control_signal:.2f}%")

        self.last_control_signal = control_signal

    def control_loop(self, stop_event):
        try:
            while not stop_event.is_set():
                if self.state == 'initializing':
                    pot_value = self.adc_reader.read_channel(self.channel)
                    degrees_value = self.map_potentiometer_value_to_degrees(pot_value)

                    self.pid_control_motor(degrees_value, self.target_position)

                    if self.at_target_position:
                        self.state = 'waiting'
                        print(f"[{self.name}] Reached initial position. Waiting to start synchronized movement.")

                elif self.state == 'waiting':
                    if self.event_start_synchronized_movement.is_set():
                        self.state = 'synchronized_movement'
                        print(f"[{self.name}] Starting synchronized movement.")
                        # Reset wave generator to synchronize start
                        if self.wave_generator:
                            self.wave_generator.start_time = self.wave_generator.current_millis()
                    else:
                        # Continue trying to reach target
                        time.sleep(0.1)

                elif self.state == 'synchronized_movement':
                    pot_value = self.adc_reader.read_channel(self.channel)
                    degrees_value = self.map_potentiometer_value_to_degrees(pot_value)

                    if self.wave_generator:
                        set_position = self.wave_generator.get_set_position()
                        self.pid_control_motor(degrees_value, set_position)
                    else:
                        print(f"[{self.name}] No wave generator. Holding position.")
                        self.motor.stop()

                else:
                    # Unknown state
                    print(f"[{self.name}] Unknown state: {self.state}. Stopping motor.")
                    self.motor.stop()
                    self.state = 'stopped'

                time.sleep(0.05)  # Control loop frequency

        except Exception as e:
            print(f"[{self.name}] Exception occurred: {e}")
        finally:
            self.motor.cleanup()
            print(f"[{self.name}] Motor GPIO cleaned up")

# ==========================
# Main Function
# ==========================
def run_motor_controller(motor_controller, stop_event):
    motor_controller.control_loop(stop_event)

def main():
    try:
        GPIO.setmode(GPIO.BOARD)

        # Create ADCReader instance for reading from ADC channels
        adc_reader = ADCReader(spi_port=0, spi_device=0, num_samples=5)

        # Shared PID configuration and settings
        config = {
            'offset': 5,  # Degrees within which to stop the motor
            'max_control_change': 10,  # Max change in control signal per loop
            'min_control_signal': 10  # Minimum control signal to move the motor
        }

        # PID constants (tune as necessary)
        Kp = 1.0
        Ki = 0.1
        Kd = 0.05

        # Motor configurations
        motors_info = [
            {
                'id': 1,
                'name': 'Motor 1',
                'in1': 7,
                'in2': 26,
                'spd': 18,
                'adc_channel': 0,
                'target_position': 165  # Midpoint of 330 degrees
            },
            {
                'id': 3,
                'name': 'Motor 3',
                'in1': 11,
                'in2': 32,
                'spd': 33,
                'adc_channel': 2,
                'target_position': 165  # Midpoint of 330 degrees
            }
        ]

        # Create and start threads for each motor controller
        stop_event = threading.Event()
        threads = []

        # Create Events for synchronization
        event_motor1_reached_position = threading.Event()
        event_motor3_reached_position = threading.Event()
        event_start_synchronized_movement = threading.Event()

        # Create SawtoothWaveGenerator instance
        wave_generator = SawtoothWaveGenerator(
            period=5000,  # 5 seconds for a full cycle
            amplitude=330,  # Full rotation within 330 degrees
            initial_angle=0,
            direction='forward',
            max_degrees=330.0
        )

        for motor_info in motors_info:
            # Create individual components for each motor
            motor = Motor(motor_info['in1'], motor_info['in2'], motor_info['spd'])
            pid_controller = PIDController(Kp, Ki, Kd)
            # SpikeFilter removed

            # Assign the correct event based on motor ID
            if motor_info['id'] == 1:
                event_reached_position = event_motor1_reached_position
            elif motor_info['id'] == 3:
                event_reached_position = event_motor3_reached_position
            else:
                event_reached_position = None

            # Create MotorController instance for each motor
            motor_controller = MotorController(
                motor=motor,
                pid_controller=pid_controller,
                adc_reader=adc_reader,
                channel=motor_info['adc_channel'],
                config=config,
                name=motor_info['name'],
                target_position=motor_info['target_position'],
                wave_generator=wave_generator,
                event_reached_position=event_reached_position,
                event_start_synchronized_movement=event_start_synchronized_movement
            )

            # Start a thread for the motor controller
            thread = threading.Thread(target=run_motor_controller, args=(motor_controller, stop_event))
            thread.start()
            threads.append(thread)

            print(f"[Main] {motor_info['name']} Controller started. Target Position: {motor_info['target_position']}°")

        print("[Main] Waiting for motors to reach initial positions...")
        # Wait for both motors to reach their initial positions
        event_motor1_reached_position.wait()
        event_motor3_reached_position.wait()

        print("[Main] Both motors have reached initial positions. Waiting for 5 seconds.")
        time.sleep(5)

        print("[Main] Starting synchronized movement.")
        wave_generator.start_time = wave_generator.current_millis()
        event_start_synchronized_movement.set()

        print("[Main] Press Ctrl+C to stop.")

        # Keep the main thread alive
        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[Main] Program stopped by user")
        stop_event.set()  # Signal threads to stop
        for thread in threads:
            thread.join()  # Wait for all threads to finish
    except Exception as e:
        print(f"[Main] Exception occurred: {e}")
    finally:
        GPIO.cleanup()
        print("[Main] GPIO cleaned up")

if __name__ == "__main__":
    main()
