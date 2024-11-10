import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import argparse

# Constants for SPI and ADC
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023
DEAD_ZONE_THRESHOLD = 50
SAWTOOTH_PERIOD = 2  # Period in seconds
MIN_ANGLE = 0
MAX_ANGLE = 330
PHASE_SHIFT = 180  # Phase shift for Motor 3 and Motor 4 in degrees

# GPIO Pins for Motors
MOTOR_PINS = {
    1: {'in1': 7, 'in2': 26, 'spd': 18, 'adc_channel': 0},
    2: {'in1': 29, 'in2': 22, 'spd': 31, 'adc_channel': 1},
    3: {'in1': 11, 'in2': 32, 'spd': 33, 'adc_channel': 2},
    4: {'in1': 12, 'in2': 13, 'spd': 35, 'adc_channel': 3},
}

# GPIO setup
GPIO.setmode(GPIO.BOARD)
for motor, pins in MOTOR_PINS.items():
    GPIO.setup(pins['in1'], GPIO.OUT)
    GPIO.setup(pins['in2'], GPIO.OUT)
    GPIO.setup(pins['spd'], GPIO.OUT)

# Set up PWM for motor speed control
motor_pwms = {motor: GPIO.PWM(pins['spd'], 1000) for motor, pins in MOTOR_PINS.items()}
for pwm in motor_pwms.values():
    pwm.start(0)

# Set up MCP3008
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

class SpikeFilter:
    def __init__(self, name, flipped=False):
        self.filter_active = False
        self.last_valid_reading = None
        self.name = name
        self.flipped = flipped

    def filter(self, new_value):
        if self.flipped:
            new_value = ADC_MAX - new_value  # Flip encoder readings

        if self.filter_active:
            if 150 <= new_value <= 700:
                print(f"[{self.name}] Discarding invalid reading during dead zone: {new_value}")
                return None
            else:
                print(f"[{self.name}] Valid reading after dead zone: {new_value}")
                self.filter_active = False
                self.last_valid_reading = new_value
                return new_value
        else:
            if self.last_valid_reading is not None and self.last_valid_reading > 950 and 150 <= new_value <= 700:
                print(f"[{self.name}] Dead zone detected: last valid {self.last_valid_reading}, new {new_value}")
                self.filter_active = True
                return None
            else:
                self.last_valid_reading = new_value
                return new_value

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, error):
        current_time = time.time()
        delta_time = current_time - self.last_time
        delta_time = max(delta_time, 0.0001)

        proportional = self.Kp * error
        self.integral += error * delta_time
        integral = self.Ki * self.integral
        derivative = self.Kd * (error - self.previous_error) / delta_time

        control_signal = proportional + integral + derivative
        self.previous_error = error
        self.last_time = current_time

        return control_signal

class MotorController:
    def __init__(self, name, in1, in2, pwm, adc_channel, target_position, phase_shift=0, flipped=False):
        self.name = name
        self.in1 = in1
        self.in2 = in2
        self.pwm = pwm
        self.adc_channel = adc_channel
        self.target_position = target_position
        self.phase_shift = phase_shift
        self.position = 0
        self.pid = PIDController(Kp=0.8, Ki=0.1, Kd=0.05)
        self.spike_filter = SpikeFilter(name, flipped=flipped)
        self.flipped = flipped
        self.start_time = None

    def read_position(self):
        raw_value = mcp.read_adc(self.adc_channel)
        filtered_value = self.spike_filter.filter(raw_value)
        if filtered_value is None:
            return self.position
        degrees = (filtered_value / ADC_MAX) * 330.0
        self.position = degrees
        return degrees

    def set_motor_direction(self, direction):
        if self.flipped:
            direction = 'backward' if direction == 'forward' else 'forward'
        GPIO.output(self.in1, GPIO.HIGH if direction == 'forward' else GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW if direction == 'forward' else GPIO.HIGH)

    def move_to_position(self, target):
        current_position = self.read_position()
        error = target - current_position
        error = error - 330 if error > 165 else error + 330 if error < -165 else error
        control_signal = self.pid.compute(error)
        direction = 'forward' if control_signal > 0 else 'backward'
        self.set_motor_direction(direction)
        speed = min(100, max(30, abs(control_signal)))
        self.pwm.ChangeDutyCycle(speed)

    def generate_sawtooth_position(self):
        if self.start_time is None:
            self.start_time = time.time()
        elapsed_time = time.time() - self.start_time
        position_in_cycle = (elapsed_time % SAWTOOTH_PERIOD) / SAWTOOTH_PERIOD
        shifted_position = (position_in_cycle * MAX_ANGLE + self.phase_shift) % MAX_ANGLE
        return 0 if shifted_position >= MAX_ANGLE else shifted_position

def main():
    parser = argparse.ArgumentParser(description="Motor control with sawtooth pattern")
    parser.add_argument("motor1_target", type=int, nargs="?", default=90)
    parser.add_argument("motor3_target", type=int, nargs="?", default=270)
    args = parser.parse_args()

    try:
        motor1 = MotorController("Motor 1", **MOTOR_PINS[1], pwm=motor_pwms[1], target_position=args.motor1_target)
        motor2 = MotorController("Motor 2", **MOTOR_PINS[2], pwm=motor_pwms[2], target_position=args.motor1_target, flipped=True)
        motor3 = MotorController("Motor 3", **MOTOR_PINS[3], pwm=motor_pwms[3], target_position=args.motor3_target)
        motor4 = MotorController("Motor 4", **MOTOR_PINS[4], pwm=motor_pwms[4], target_position=args.motor3_target, phase_shift=PHASE_SHIFT, flipped=True)

        while True:
            m1_target = motor1.generate_sawtooth_position()
            m3_target = motor3.generate_sawtooth_position()
            motor1.move_to_position(m1_target)
            motor2.move_to_position(m1_target)
            motor3.move_to_position(m3_target)
            motor4.move_to_position(m3_target)
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    finally:
        for pwm in motor_pwms.values():
            pwm.stop()
        GPIO.cleanup()
        print("GPIO cleaned up")

if __name__ == "__main__":
    main()
