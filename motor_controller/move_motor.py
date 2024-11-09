import RPi.GPIO as GPIO
import threading
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import math

# ==========================
# ADCReader Class
# ==========================
class ADCReader:
    def __init__(self, spi_port=0, spi_device=0, num_samples=1):
        self.mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(spi_port, spi_device))
        self.lock = threading.Lock()
        self.num_samples = num_samples
        self.readings = {channel: [] for channel in range(8)}

    def read_channel(self, channel):
        with self.lock:
            adc_value = self.mcp.read_adc(channel)
            degrees_330 = (adc_value / 1023.0) * 330.0
            print(f"[ADCReader] Channel {channel} ADC Value: {adc_value}, Degrees_330: {degrees_330:.2f}")
            return degrees_330

# ==========================
# Motor Class
# ==========================
class Motor:
    def __init__(self, in1_pin, in2_pin, spd_pin, pwm_frequency=1000):
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.spd_pin = spd_pin
        self.pwm_frequency = pwm_frequency

        GPIO.setup(self.in1_pin, GPIO.OUT)
        GPIO.setup(self.in2_pin, GPIO.OUT)
        GPIO.setup(self.spd_pin, GPIO.OUT)

        self.pwm = GPIO.PWM(self.spd_pin, self.pwm_frequency)
        self.pwm.start(0)

    def set_speed(self, duty_cycle):
        duty_cycle = max(0, min(100, duty_cycle))
        self.pwm.ChangeDutyCycle(duty_cycle)
        print(f"[Motor] Set speed to {duty_cycle:.2f}% duty cycle")

    def forward(self, duty_cycle):
        GPIO.output(self.in1_pin, GPIO.LOW)
        GPIO.output(self.in2_pin, GPIO.HIGH)
        self.set_speed(duty_cycle)
        print("[Motor] Moving forward")

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
    def __init__(self, Kp, Ki, Kd, set_point=0, integral_limit=1000, max_angle=330.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.set_point = set_point
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()
        self.integral_limit = integral_limit
        self.max_angle = max_angle

    def compute(self, current_value):
        current_time = time.time()
        delta_time = current_time - self.last_time
        if delta_time <= 0.0:
            delta_time = 0.0001

        # Proper error wrapping calculation
        error = self.calculate_wrapped_error(self.set_point, current_value)
        self.integral += error * delta_time
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))

        derivative = (error - self.previous_error) / delta_time
        control_signal = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # Clamp control signal to be within the motor's operable range
        control_signal = max(10, min(100, control_signal))  # Ensure a minimum to overcome stiction

        print(f"[PIDController] Error: {error:.2f}, Integral: {self.integral:.2f}, Derivative: {derivative:.2f}, "
              f"Control Signal: {control_signal:.2f}")

        self.previous_error = error
        self.last_time = current_time

        return control_signal

    def calculate_wrapped_error(self, set_point, current_value):
        error = set_point - current_value
        if error > self.max_angle / 2:
            error -= self.max_angle
        elif error < -self.max_angle / 2:
            error += self.max_angle
        return error

# ==========================
# MotorController Class
# ==========================
class MotorController:
    def __init__(self, motor, pid_controller, adc_reader, channel, config,
                 name='Motor', target_position=0, max_angle=330.0,
                 event_reached_position=None):
        self.motor = motor
        self.pid_controller = pid_controller
        self.adc_reader = adc_reader
        self.channel = channel
        self.config = config
        self.name = name
        self.target_position = target_position
        self.max_angle = max_angle
        self.event_reached_position = event_reached_position
        self.state = 'moving_to_target'
        self.at_target_position = False
        self.last_control_signal = 0

    def pid_control_motor(self, degrees_value):
        OFFSET = self.config['offset']
        max_control_change = self.config['max_control_change']
        MIN_CONTROL_SIGNAL = self.config['min_control_signal']

        error = self.pid_controller.calculate_wrapped_error(self.target_position, degrees_value)
        
        if abs(error) <= OFFSET:
            self.motor.stop()
            if not self.at_target_position:
                self.at_target_position = True
                if self.event_reached_position:
                    self.event_reached_position.set()
                print(f"[{self.name}] Reached target position within offset: {degrees_value:.2f}°")
            return

        self.pid_controller.set_point = self.target_position
        control_signal = self.pid_controller.compute(degrees_value)

        control_signal_change = control_signal - self.last_control_signal
        if control_signal_change > max_control_change:
            control_signal = self.last_control_signal + max_control_change
        elif control_signal_change < -max_control_change:
            control_signal = self.last_control_signal - max_control_change

        control_signal = max(MIN_CONTROL_SIGNAL, min(100, control_signal))

        self.motor.forward(control_signal)
        print(f"[{self.name}] Moving forward: Potentiometer Value: {degrees_value:.2f}°, "
              f"Error: {error:.2f}°, Control Signal: {control_signal:.2f}%")

        self.last_control_signal = control_signal

    def control_loop(self, stop_event):
        try:
            while not stop_event.is_set():
                if self.state == 'moving_to_target':
                    pot_value = self.adc_reader.read_channel(self.channel)
                    degrees_value = self.map_potentiometer_value_to_degrees(pot_value)

                    self.pid_control_motor(degrees_value)

                    if self.at_target_position:
                        self.state = 'stopped'
                        print(f"[{self.name}] Motor has stopped at target position.")
                elif self.state == 'stopped':
                    time.sleep(0.1)
                else:
                    self.motor.stop()
                    self.state = 'stopped'

                time.sleep(0.05)

        except Exception as e:
            print(f"[{self.name}] Exception occurred: {e}")
        finally:
            self.motor.cleanup()
            print(f"[{self.name}] Motor GPIO cleaned up")

    def map_potentiometer_value_to_degrees(self, value):
        degrees = value
        print(f"[{self.name}] Current Position: {degrees:.2f} degrees")
        return degrees

# ==========================
# Main Function
# ==========================
def run_motor_controller(motor_controller, stop_event):
    motor_controller.control_loop(stop_event)

def main():
    try:
        GPIO.setmode(GPIO.BOARD)

        adc_reader = ADCReader(spi_port=0, spi_device=0, num_samples=1)

        config = {
            'offset': 5,
            'max_control_change': 10,
            'min_control_signal': 10
        }

        Kp = 0.06
        Ki = 0.1
        Kd = 0.05

        motor_info = {
            'id': 1,
            'name': 'Motor 1',
            'in1': 7,
            'in2': 26,
            'spd': 18,
            'adc_channel': 0,
            'target_position': 165
        }

        motor1 = Motor(motor_info['in1'], motor_info['in2'], motor_info['spd'])
        pid_controller1 = PIDController(Kp, Ki, Kd, set_point=motor_info['target_position'], max_angle=330.0)
        motor_controller1 = MotorController(
            motor=motor1,
            pid_controller=pid_controller1,
            adc_reader=adc_reader,
            channel=motor_info['adc_channel'],
            config=config,
            name=motor_info['name'],
            target_position=motor_info['target_position'],
            max_angle=330.0
        )

        stop_event = threading.Event()
        thread_motor1 = threading.Thread(target=run_motor_controller, args=(motor_controller1, stop_event))
        thread_motor1.start()
        print(f"[Main] {motor_info['name']} Controller started. Target Position: {motor_info['target_position']}°")

        print("[Main] Press Ctrl+C to stop.")

        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[Main] Program stopped by user")
        stop_event.set()
        thread_motor1.join()
    except Exception as e:
        print(f"[Main] Exception occurred: {e}")
    finally:
        GPIO.cleanup()
        print("[Main] GPIO cleaned up")

if __name__ == "__main__":
    main()
