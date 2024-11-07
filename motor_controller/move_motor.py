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
# PIDController Class - Improved
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
# MotorController Class - Improved
# ==========================
class MotorController:
    def __init__(self, motor, pid_controller, adc_reader, channel, spike_filter, config, name='Motor', initial_position=0, target_position=90):
        self.motor = motor
        self.pid_controller = pid_controller
        self.adc_reader = adc_reader
        self.channel = channel
        self.spike_filter = spike_filter
        self.config = config
        self.name = name  # For logging purposes

        # Initialization parameters
        self.initial_position = initial_position
        self.target_position = target_position
        self.initialized = False

    def map_potentiometer_value_to_degrees(self, value):
        degrees = (value / 1023) * 360
        return degrees % 360

    def calculate_error(self, set_position, current_angle):
        # Calculate minimal angular difference
        error = (set_position - current_angle + 180) % 360 - 180
        return error

    def pid_control_motor(self, degrees_value, set_position, initialization=False):
        OFFSET = self.config['offset']
        max_control_change = self.config['max_control_change']
        MIN_CONTROL_SIGNAL = 5  # Reduced minimum control signal to allow for finer movements

        if degrees_value is None:
            print(f"[{self.name}] degrees_value is None, cannot compute PID control.")
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

            print(f"[{self.name}] [Init] Moving {dir_text}: Potentiometer Value: {degrees_value:.2f}°, Error: {error:.2f}°, Control Signal: {control_signal:.2f}%, Set Position: {set_position:.2f}°")
        else:
            # Normal operation
            if abs(error) <= OFFSET:
                self.motor.set_speed(0)
                self.motor.stop()
                print(f"[{self.name}] Motor stopped at target: {degrees_value:.2f} degrees")
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

                print(f"[{self.name}] Moving {dir_text}: Potentiometer Value: {degrees_value:.2f}°, Error: {error:.2f}°, Control Signal: {control_signal:.2f}%, Set Position: {set_position:.2f}°")

        # Update last control signal
        self.last_control_signal = control_signal

# ==========================
# Updated Configuration
# ==========================
config = {
    'offset': 2,  # Reduced offset for better stopping precision
    'max_control_change': 3,  # Reduced max control change for finer adjustments
}

# PID constants (tune as necessary)
Kp = 0.7   # Increased Proportional gain for faster response
Ki = 0.1   # Slightly increased Integral gain to minimize steady-state error
Kd = 0.2   # Increased Derivative gain to reduce overshoot

# ==========================
# Main Function
# ==========================
def run_motor_controller(motor_controller, stop_event):
    motor_controller.control_loop(stop_event)

def main():
    try:
        # Set up argument parser
        parser = argparse.ArgumentParser(description='Single Motor Control Program')
        parser.add_argument('--target', type=float, default=90.0, help='Target position in degrees (0-360)')
        args = parser.parse_args()

        # Extract the target position
        target_position = args.target

        # Validate target position
        if not (0 <= target_position <= 360):
            raise ValueError("Target position must be between 0 and 360 degrees.")

        # Set up GPIO
        GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering

        # Create ADCReader instance
        adc_reader = ADCReader(spi_port=0, spi_device=0)

        # Define Motor configuration
        motor_info = {
            'name': 'Motor 1',
            'in1': 7,
            'in2': 26,
            'spd': 18,
            'adc_channel': 0,
            'initial_position': target_position  # Using target_position for simplicity
        }

        # Create Motor instance
        motor = Motor(motor_info['in1'], motor_info['in2'], motor_info['spd'])

        # Create PIDController instance
        pid_controller = PIDController(Kp, Ki, Kd)

        # Create SpikeFilter instance with appropriate thresholds
        spike_filter = SpikeFilter(high_threshold=300, low_threshold=30)

        # Create MotorController instance
        motor_controller = MotorController(
            motor=motor,
            pid_controller=pid_controller,
            adc_reader=adc_reader,
            channel=motor_info['adc_channel'],
            spike_filter=spike_filter,
            config=config,
            name=motor_info['name'],
            initial_position=motor_info['initial_position'],
            target_position=target_position
        )

        # Create and start thread for MotorController
        stop_event = threading.Event()
        t = threading.Thread(target=run_motor_controller, args=(motor_controller, stop_event))
        t.start()

        print(f"[Main] Motor Controller started. Target Position: {target_position}°")
        print("[Main] Press Ctrl+C to stop.")

        # Wait for KeyboardInterrupt to stop
        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[Main] Program stopped by user")
        stop_event.set()  # Signal thread to stop
        t.join()  # Wait for thread to finish
    except Exception as e:
        print(f"[Main] Exception occurred: {e}")
    finally:
        GPIO.cleanup()
        print("[Main] GPIO cleaned up")

if __name__ == "__main__":
    main()

