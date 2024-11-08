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
            adc_value = self.mcp.read_adc(channel)
            print(f"[ADCReader] Channel {channel} ADC Value: {adc_value}")
            return adc_value

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
        error = ((error + 180) % 360) - 180

        self.integral += error * delta_time
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))

        derivative = (error - self.previous_error) / delta_time
        control_signal = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        print(f"[PIDController] Error: {error:.2f}, Integral: {self.integral:.2f}, Derivative: {derivative:.2f}, Control Signal: {control_signal:.2f}")

        self.previous_error = error
        self.last_time = current_time

        return control_signal

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
            if new_value > self.high_threshold or new_value < self.low_threshold:
                print(f"[SpikeFilter] Exiting dead zone with value: {new_value}")
                self.filter_active = False
                self.last_valid_reading = new_value
                return new_value
            else:
                print(f"[SpikeFilter] Discarding invalid reading in dead zone: {new_value}")
                return None
        else:
            if self.last_valid_reading is not None:
                if self.last_valid_reading > self.high_threshold and new_value < self.low_threshold:
                    print(f"[SpikeFilter] Entering dead zone. Last valid: {self.last_valid_reading}, New: {new_value}")
                    self.filter_active = True
                    return None
            self.last_valid_reading = new_value
            return new_value

# ==========================
# MotorController Class
# ==========================
class MotorController:
    def __init__(self, motor, pid_controller, adc_reader, channel, spike_filter, config, name='Motor', initial_position=0, target_position=90):
        self.motor = motor
        self.pid_controller = pid_controller
        self.adc_reader = adc_reader
        self.channel = channel
        self.spike_filter = spike_filter
        self.config = config
        self.name = name
        self.initial_position = initial_position
        self.target_position = target_position
        self.initialized = False

    def map_potentiometer_value_to_degrees(self, value):
        degrees = (value / 1023) * 360
        print(f"[{self.name}] Mapped ADC Value {value} to {degrees:.2f} degrees")
        return degrees % 360

    def calculate_error(self, set_position, current_angle):
        error = (set_position - current_angle + 180) % 360 - 180
        print(f"[{self.name}] Calculated Error: {error:.2f}° (Set Position: {set_position}°, Current Angle: {current_angle}°)")
        return error

    def pid_control_motor(self, degrees_value, set_position, initialization=False):
        OFFSET = self.config['offset']
        max_control_change = self.config['max_control_change']
        MIN_CONTROL_SIGNAL = 10

        if degrees_value is None:
            print(f"[{self.name}] degrees_value is None, cannot compute PID control.")
            self.motor.stop()
            return

        error = self.calculate_error(set_position, degrees_value)
        self.pid_controller.set_point = set_position
        control_signal = self.pid_controller.compute(degrees_value)

        if not hasattr(self, 'last_control_signal'):
            self.last_control_signal = 0

        control_signal_change = control_signal - self.last_control_signal
        if abs(control_signal_change) > max_control_change:
            control_signal = self.last_control_signal + max_control_change * (1 if control_signal_change > 0 else -1)

        control_signal = max(-100, min(100, control_signal))

        if abs(error) <= OFFSET:
            self.motor.set_speed(0)
            self.motor.stop()
            print(f"[{self.name}] Motor stopped at target: {degrees_value:.2f} degrees")
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
                pot_value = self.adc_reader.read_channel(self.channel)
                filtered_pot_value = self.spike_filter.filter(pot_value)
                degrees_value = None if filtered_pot_value is None else self.map_potentiometer_value_to_degrees(filtered_pot_value)

                if degrees_value is not None:
                    set_position = self.target_position
                    self.pid_control_motor(degrees_value, set_position, initialization=False)
                else:
                    print(f"[{self.name}] degrees_value is None, holding position.")
                    self.motor.stop()

                time.sleep(0.1)

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
        adc_reader = ADCReader(spi_port=0, spi_device=0)

        # Shared PID configuration and settings
        config = {
            'offset': 5,  # Degrees within which to stop the motor
            'max_control_change': 5,  # Max change in control signal per loop
        }

        # PID constants (tune as necessary)
        Kp = 0.5
        Ki = 0.05
        Kd = 0.15

        # Motor configurations
        motors_info = [
            {
                'id': 1,
                'name': 'Motor 1',
                'in1': 7,
                'in2': 26,
                'spd': 18,
                'adc_channel': 0,
                'target_position': 90  # Target for Motor 1
            },
            {
                'id': 3,
                'name': 'Motor 3',
                'in1': 11,
                'in2': 32,
                'spd': 33,
                'adc_channel': 2,
                'target_position': 270  # Target for Motor 3
            }
        ]

        # Create and start threads for each motor controller
        stop_event = threading.Event()
        threads = []

        for motor_info in motors_info:
            # Create individual components for each motor
            motor = Motor(motor_info['in1'], motor_info['in2'], motor_info['spd'])
            pid_controller = PIDController(Kp, Ki, Kd)
            spike_filter = SpikeFilter(high_threshold=300, low_threshold=30)

            # Create MotorController instance for each motor
            motor_controller = MotorController(
                motor=motor,
                pid_controller=pid_controller,
                adc_reader=adc_reader,
                channel=motor_info['adc_channel'],
                spike_filter=spike_filter,
                config=config,
                name=motor_info['name'],
                target_position=motor_info['target_position']
            )

            # Start a thread for the motor controller
            thread = threading.Thread(target=run_motor_controller, args=(motor_controller, stop_event))
            thread.start()
            threads.append(thread)

            print(f"[Main] {motor_info['name']} Controller started. Target Position: {motor_info['target_position']}°")

        print("[Main] Press Ctrl+C to stop.")

        # Wait for KeyboardInterrupt to stop
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

