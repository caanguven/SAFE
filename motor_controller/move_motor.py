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
PHASE_SHIFT = 180  # Phase shift for Motor 3 and Motor 2 in degrees

# Motor configuration
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
        'id': 2,
        'name': 'Motor 2',
        'in1': 29,
        'in2': 22,
        'spd': 31,
        'adc_channel': 1,
    },
    {
        'id': 3,
        'name': 'Motor 3',
        'in1': 11,
        'in2': 32,
        'spd': 33,
        'adc_channel': 2,
    },
    {
        'id': 4,
        'name': 'Motor 4',
        'in1': 12,
        'in2': 13,
        'spd': 35,
        'adc_channel': 3,
    },
]

# GPIO setup
GPIO.setmode(GPIO.BOARD)

# Initialize MCP3008 for ADC
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

class SpikeFilter:
    def __init__(self, name):
        self.filter_active = False
        self.last_valid_reading = None
        self.name = name  # For debugging purposes

    def filter(self, new_value):
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
    def __init__(self, motor_info, target_position, phase_shift=0, flipped=False):
        self.name = motor_info['name']
        self.in1 = motor_info['in1']
        self.in2 = motor_info['in2']
        self.pwm_pin = motor_info['spd']
        self.adc_channel = motor_info['adc_channel']
        self.target_position = target_position
        self.phase_shift = phase_shift
        self.position = 0
        self.last_valid_position = None
        self.pid = PIDController(Kp=0.8, Ki=0.1, Kd=0.05)
        self.start_time = None
        self.spike_filter = SpikeFilter(self.name)
        self.flipped = flipped

        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pwm_pin, 1000)
        self.pwm.start(0)

    def read_position(self):
        raw_value = mcp.read_adc(self.adc_channel)
        filtered_value = self.spike_filter.filter(raw_value)
        if filtered_value is None:
            return self.last_valid_position if self.last_valid_position is not None else 0
        
        degrees = (filtered_value / ADC_MAX) * 330.0
        if self.flipped:
            degrees = 330.0 - degrees  # Flip encoder reading if necessary
        self.last_valid_position = degrees
        return degrees

    def set_motor_direction(self, direction):
        if self.flipped:
            direction = 'backward' if direction == 'forward' else 'forward'
        
        if direction == 'forward':
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)

    def move_to_position(self, target):
        current_position = self.read_position()
        error = target - current_position
        if error > 165:
            error -= 330
        elif error < -165:
            error += 330

        control_signal = self.pid.compute(error)

        if abs(error) <= 2:
            self.stop_motor()
            return True

        direction = 'forward' if control_signal > 0 else 'backward'
        self.set_motor_direction(direction)
        speed = min(100, max(30, abs(control_signal)))
        self.pwm.ChangeDutyCycle(speed)
        return False

    def stop_motor(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)

    def generate_sawtooth_position(self):
        if self.start_time is None:
            self.start_time = time.time()
            
        elapsed_time = time.time() - self.start_time
        position_in_cycle = (elapsed_time % SAWTOOTH_PERIOD) / SAWTOOTH_PERIOD
        shifted_position = (position_in_cycle * MAX_ANGLE + self.phase_shift) % MAX_ANGLE
        
        if shifted_position >= MAX_ANGLE:
            shifted_position = 0
        return shifted_position

def main():
    parser = argparse.ArgumentParser(description="Motor control with sawtooth pattern for Quadruped Robot")
    parser.add_argument("--motor1_target", type=int, default=90)
    parser.add_argument("--motor2_target", type=int, default=270)
    parser.add_argument("--motor3_target", type=int, default=270)
    parser.add_argument("--motor4_target", type=int, default=90)
    args = parser.parse_args()

    try:
        motors = [
            MotorController(motors_info[0], args.motor1_target, phase_shift=0, flipped=False),
            MotorController(motors_info[1], args.motor2_target, phase_shift=PHASE_SHIFT, flipped=True),
            MotorController(motors_info[2], args.motor3_target, phase_shift=PHASE_SHIFT, flipped=False),
            MotorController(motors_info[3], args.motor4_target, phase_shift=0, flipped=True)
        ]

        print("Starting initial calibration...")
        calibration_start = time.time()
        while True:
            calibration_done = all(motor.move_to_position(motor.target_position) for motor in motors)

            if calibration_done:
                break
            if time.time() - calibration_start > 30:
                print("\nCalibration timeout! Check motor connections.")
                raise TimeoutError("Calibration timeout")
                
            time.sleep(0.05)

        print("\nCalibration complete. Starting sawtooth pattern...")
        while True:
            for motor in motors:
                target_position = motor.generate_sawtooth_position()
                motor.move_to_position(target_position)

                # Print current status
                current_position = motor.read_position()
                print(f"{motor.name} - Target: {target_position:.1f}°, Current: {current_position:.1f}°")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        for motor in motors:
            motor.stop_motor()
            motor.pwm.stop()
        GPIO.cleanup()
        print("GPIO cleaned up")

if __name__ == "__main__":
    main()
