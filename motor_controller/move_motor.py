  
import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import argparse

#Constants for SPI and ADC
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023
DEAD_ZONE_THRESHOLD = 50
SAWTOOTH_PERIOD = 2  # Period in seconds
MIN_ANGLE = 0
MAX_ANGLE = 330
PHASE_SHIFT = 180  # Phase shift for Motor 3 in degrees

# GPIO Pins for Motor 1
MOTOR1_IN1 = 7
MOTOR1_IN2 = 26
MOTOR1_SPD = 18
MOTOR1_ADC_CHANNEL = 0

# GPIO Pins for Motor 3
MOTOR3_IN1 = 11
MOTOR3_IN2 = 32
MOTOR3_SPD = 33
MOTOR3_ADC_CHANNEL = 2

# GPIO setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(MOTOR1_IN1, GPIO.OUT)
GPIO.setup(MOTOR1_IN2, GPIO.OUT)
GPIO.setup(MOTOR1_SPD, GPIO.OUT)
GPIO.setup(MOTOR3_IN1, GPIO.OUT)
GPIO.setup(MOTOR3_IN2, GPIO.OUT)
GPIO.setup(MOTOR3_SPD, GPIO.OUT)

# Set up PWM for motor speed control
motor1_pwm = GPIO.PWM(MOTOR1_SPD, 1000)
motor1_pwm.start(0)
motor3_pwm = GPIO.PWM(MOTOR3_SPD, 1000)
motor3_pwm.start(0)

# Set up MCP3008
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

class SpikeFilter:
    def __init__(self, name):
        self.filter_active = False
        self.last_valid_reading = None
        self.name = name  # For debugging purposes

    def filter(self, new_value):
        # If the filter is active, we are in the dead zone
        if self.filter_active:
            # Discard readings between 150 and 700
            if 150 <= new_value <= 700:
                print(f"[{self.name}] Discarding invalid reading during dead zone: {new_value}")
                return None
            else:
                # Valid reading after dead zone
                print(f"[{self.name}] Valid reading after dead zone: {new_value}")
                self.filter_active = False
                self.last_valid_reading = new_value
                return new_value
        else:
            # Not currently filtering
            if self.last_valid_reading is not None and self.last_valid_reading > 950 and 150 <= new_value <= 700:
                # Sudden drop into dead zone detected
                print(f"[{self.name}] Dead zone detected: last valid {self.last_valid_reading}, new {new_value}")
                self.filter_active = True
                return None
            else:
                # Valid reading
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
        self.pid = PIDController(Kp=0.8, Ki=0.1, Kd=0.05)
        self.start_time = None
        self.spike_filter = SpikeFilter(name)
        
    [Previous MotorController methods remain exactly the same]

def main():
    parser = argparse.ArgumentParser(description="Motor control with sawtooth pattern")
    parser.add_argument("initial_position", type=int, nargs="?", default=90,
                       help="Initial position for motors 1 and 4")
    parser.add_argument("shifted_position", type=int, nargs="?", default=270,
                       help="Initial position for motors 2 and 3")
    args = parser.parse_args()

    try:
        # Initialize all motors
        # Motors 1 and 4 are in sync with no phase shift
        motor1 = MotorController("Motor 1", MOTOR1_IN1, MOTOR1_IN2, motor1_pwm, 
                               MOTOR1_ADC_CHANNEL, args.initial_position, phase_shift=0)
        motor4 = MotorController("Motor 4", MOTOR4_IN1, MOTOR4_IN2, motor4_pwm, 
                               MOTOR4_ADC_CHANNEL, args.initial_position, phase_shift=0)
        
        # Motors 2 and 3 are in sync with PHASE_SHIFT relative to motors 1/4
        motor2 = MotorController("Motor 2", MOTOR2_IN1, MOTOR2_IN2, motor2_pwm, 
                               MOTOR2_ADC_CHANNEL, args.shifted_position, phase_shift=PHASE_SHIFT)
        motor3 = MotorController("Motor 3", MOTOR3_IN1, MOTOR3_IN2, motor3_pwm, 
                               MOTOR3_ADC_CHANNEL, args.shifted_position, phase_shift=PHASE_SHIFT)

        # Initial calibration
        print("Starting initial calibration...")
        calibration_start = time.time()
        while True:
            m1_done = motor1.move_to_position(args.initial_position)
            m2_done = motor2.move_to_position(args.shifted_position)
            m3_done = motor3.move_to_position(args.shifted_position)
            m4_done = motor4.move_to_position(args.initial_position)
            
            # Print calibration progress
            print(f"\rCalibrating - "
                  f"M1: {motor1.read_position():.1f}° / {args.initial_position}°, "
                  f"M2: {motor2.read_position():.1f}° / {args.shifted_position}°, "
                  f"M3: {motor3.read_position():.1f}° / {args.shifted_position}°, "
                  f"M4: {motor4.read_position():.1f}° / {args.initial_position}°", end='')
            
            if m1_done and m2_done and m3_done and m4_done:
                break
                
            # Timeout after 30 seconds
            if time.time() - calibration_start > 30:
                print("\nCalibration timeout! Check motor connections.")
                raise TimeoutError("Calibration timeout")
                
            time.sleep(0.05)

        print("\nCalibration complete. Stopping for 5 seconds...")
        for motor in [motor1, motor2, motor3, motor4]:
            motor.stop_motor()
        time.sleep(5)

        print("Starting sawtooth pattern...")
        while True:
            # Generate and apply positions for all motors
            for motor in [motor1, motor2, motor3, motor4]:
                target = motor.generate_sawtooth_position()
                motor.move_to_position(target)
                pos = motor.read_position()
                print(f"{motor.name} - Target: {target:.1f}°, Current: {pos:.1f}°")
            
            # Print phase differences
            m1_pos = motor1.read_position()
            m2_pos = motor2.read_position()
            m3_pos = motor3.read_position()
            m4_pos = motor4.read_position()
            print(f"Phase Difference (M2-M1): {abs(m2_pos - m1_pos):.1f}°")
            print(f"Phase Difference (M3-M1): {abs(m3_pos - m1_pos):.1f}°")
            print("---")
            
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        # Stop all motors and cleanup
        for motor in [motor1, motor2, motor3, motor4]:
            motor.stop_motor()
        for pwm in [motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm]:
            pwm.stop()
        GPIO.cleanup()
        print("GPIO cleaned up")

if __name__ == "__main__":
    main()