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

# GPIO Pins for Motor 1
MOTOR1_IN1 = 7
MOTOR1_IN2 = 26
MOTOR1_SPD = 18
MOTOR1_ADC_CHANNEL = 0

# GPIO Pins for Motor 2
MOTOR2_IN1 = 29
MOTOR2_IN2 = 22
MOTOR2_SPD = 31
MOTOR2_ADC_CHANNEL = 1

# GPIO Pins for Motor 3
MOTOR3_IN1 = 11
MOTOR3_IN2 = 32
MOTOR3_SPD = 33
MOTOR3_ADC_CHANNEL = 2

# GPIO Pins for Motor 4
MOTOR4_IN1 = 12
MOTOR4_IN2 = 13
MOTOR4_SPD = 35
MOTOR4_ADC_CHANNEL = 3

# GPIO setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup([MOTOR1_IN1, MOTOR1_IN2, MOTOR1_SPD, MOTOR2_IN1, MOTOR2_IN2, MOTOR2_SPD,
            MOTOR3_IN1, MOTOR3_IN2, MOTOR3_SPD, MOTOR4_IN1, MOTOR4_IN2, MOTOR4_SPD], GPIO.OUT)

# Set up PWM for motor speed control
motor1_pwm = GPIO.PWM(MOTOR1_SPD, 1000)
motor2_pwm = GPIO.PWM(MOTOR2_SPD, 1000)
motor3_pwm = GPIO.PWM(MOTOR3_SPD, 1000)
motor4_pwm = GPIO.PWM(MOTOR4_SPD, 1000)

for pwm in [motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm]:
    pwm.start(0)

# Set up MCP3008
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

class SpikeFilter:
    def __init__(self, name):
        self.filter_active = False
        self.last_valid_reading = None
        self.name = name

    def filter(self, new_value):
        if self.filter_active:
            if 150 <= new_value <= 700:
                return None
            else:
                self.filter_active = False
                self.last_valid_reading = new_value
                return new_value
        else:
            if self.last_valid_reading is not None and self.last_valid_reading > 950 and 150 <= new_value <= 700:
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
        if delta_time <= 0.0:
            delta_time = 0.0001

        proportional = self.Kp * error
        self.integral += error * delta_time
        integral = self.Ki * self.integral
        derivative = self.Kd * (error - self.previous_error) / delta_time

        control_signal = proportional + integral + derivative
        self.previous_error = error
        self.last_time = current_time

        return control_signal

class MotorController:
    def __init__(self, name, in1, in2, pwm, adc_channel, target_position, phase_shift=0):
        self.name = name
        self.in1 = in1
        self.in2 = in2
        self.pwm = pwm
        self.adc_channel = adc_channel
        self.target_position = target_position
        self.phase_shift = phase_shift
        self.position = 0
        self.in_dead_zone = False
        self.last_valid_position = None
        self.pid = PIDController(Kp=0.8, Ki=0.05, Kd=0.05)  # Adjusted PID values for stability
        self.start_time = None
        self.spike_filter = SpikeFilter(name)

    def read_position(self):
        raw_value = mcp.read_adc(self.adc_channel)
        filtered_value = self.spike_filter.filter(raw_value)
        
        if filtered_value is None:
            return self.last_valid_position if self.last_valid_position is not None else 0
        
        degrees = (filtered_value / ADC_MAX) * 330.0
        self.last_valid_position = degrees
        return degrees

    def set_motor_direction(self, direction):
        GPIO.output(self.in1, GPIO.HIGH if direction == 'forward' else GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW if direction == 'forward' else GPIO.HIGH)

    def move_to_position(self, target):
        current_position = self.read_position()
        error = target - current_position

        if error > 165:
            error -= 330
        elif error < -165:
            error += 330

        control_signal = self.pid.compute(error)

        if abs(error) <= 2:  # Tolerance
            self.stop_motor()
            return True
        
        self.set_motor_direction('forward' if control_signal > 0 else 'backward')
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
        return shifted_position if shifted_position < MAX_ANGLE else 0

def main():
    parser = argparse.ArgumentParser(description="Motor control with sawtooth pattern")
    parser.add_argument("motor1_target", type=int, nargs="?", default=90)
    parser.add_argument("motor3_target", type=int, nargs="?", default=270)
    args = parser.parse_args()

    try:
        motor1 = MotorController("Motor 1", MOTOR1_IN1, MOTOR1_IN2, motor1_pwm, 
                                 MOTOR1_ADC_CHANNEL, args.motor1_target, phase_shift=0)
        motor2 = MotorController("Motor 2", MOTOR2_IN1, MOTOR2_IN2, motor2_pwm, 
                                 MOTOR2_ADC_CHANNEL, args.motor3_target, phase_shift=PHASE_SHIFT)
        motor3 = MotorController("Motor 3", MOTOR3_IN1, MOTOR3_IN2, motor3_pwm, 
                                 MOTOR3_ADC_CHANNEL, args.motor3_target, phase_shift=PHASE_SHIFT)
        motor4 = MotorController("Motor 4", MOTOR4_IN1, MOTOR4_IN2, motor4_pwm, 
                                 MOTOR4_ADC_CHANNEL, args.motor1_target, phase_shift=0)

        calibration_start = time.time()
        while True:
            m1_done = motor1.move_to_position(args.motor1_target)
            m2_done = motor2.move_to_position(args.motor3_target)
            m3_done = motor3.move_to_position(args.motor3_target)
            m4_done = motor4.move_to_position(args.motor1_target)
            
            m1_pos = motor1.read_position()
            m2_pos = motor2.read_position()
            m3_pos = motor3.read_position()
            m4_pos = motor4.read_position()
            print(f"\rCalibrating - M1: {m1_pos:.1f}° / {args.motor1_target}°, "
                  f"M2: {m2_pos:.1f}° / {args.motor3_target}°, "
                  f"M3: {m3_pos:.1f}° / {args.motor3_target}°, "
                  f"M4: {m4_pos:.1f}° / {args.motor1_target}°", end='')
            
            if m1_done and m2_done and m3_done and m4_done:
                break
                
            if time.time() - calibration_start > 30:
                print("\nCalibration timeout! Check motor connections.")
                raise TimeoutError("Calibration timeout")
                
            time.sleep(0.05)

        motor1.stop_motor()
        motor2.stop_motor()
        motor3.stop_motor()
        motor4.stop_motor()
        time.sleep(5)

        while True:
            m1_target = motor1.generate_sawtooth_position()
            m3_target = motor3.generate_sawtooth_position()
            
            m2_target = m3_target
            m4_target = m1_target
            
            motor1.move_to_position(m1_target)
            motor2.move_to_position(m2_target)
            motor3.move_to_position(m3_target)
            motor4.move_to_position(m4_target)
            
            m1_pos = motor1.read_position()
            m2_pos = motor2.read_position()
            m3_pos = motor3.read_position()
            m4_pos = motor4.read_position()
            print(f"Motor 1 - Target: {m1_target:.1f}°, Current: {m1_pos:.1f}°")
            print(f"Motor 2 - Target: {m2_target:.1f}°, Current: {m2_pos:.1f}°")
            print(f"Motor 3 - Target: {m3_target:.1f}°, Current: {m3_pos:.1f}°")
            print(f"Motor 4 - Target: {m4_target:.1f}°, Current: {m4_pos:.1f}°")
            print(f"Phase Difference between M3 and M1: {abs(m3_pos - m1_pos):.1f}°")
            
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        motor1.stop_motor()
        motor2.stop_motor()
        motor3.stop_motor()
        motor4.stop_motor()
        motor1_pwm.stop()
        motor2_pwm.stop()
        motor3_pwm.stop()
        motor4_pwm.stop()
        GPIO.cleanup()
        print("GPIO cleaned up")

if __name__ == "__main__":
    main()
