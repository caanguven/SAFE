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
PHASE_SHIFT = 180  # Phase shift between motor groups in degrees

# GPIO Pins for all motors
MOTOR1_IN1 = 7
MOTOR1_IN2 = 26
MOTOR1_SPD = 18
MOTOR1_ADC_CHANNEL = 0

MOTOR2_IN1 = 29
MOTOR2_IN2 = 22
MOTOR2_SPD = 31
MOTOR2_ADC_CHANNEL = 1

MOTOR3_IN1 = 11
MOTOR3_IN2 = 32
MOTOR3_SPD = 33
MOTOR3_ADC_CHANNEL = 2

MOTOR4_IN1 = 12
MOTOR4_IN2 = 13
MOTOR4_SPD = 35
MOTOR4_ADC_CHANNEL = 3

# GPIO setup
GPIO.setmode(GPIO.BOARD)
# Motor 1
GPIO.setup(MOTOR1_IN1, GPIO.OUT)
GPIO.setup(MOTOR1_IN2, GPIO.OUT)
GPIO.setup(MOTOR1_SPD, GPIO.OUT)
# Motor 2
GPIO.setup(MOTOR2_IN1, GPIO.OUT)
GPIO.setup(MOTOR2_IN2, GPIO.OUT)
GPIO.setup(MOTOR2_SPD, GPIO.OUT)
# Motor 3
GPIO.setup(MOTOR3_IN1, GPIO.OUT)
GPIO.setup(MOTOR3_IN2, GPIO.OUT)
GPIO.setup(MOTOR3_SPD, GPIO.OUT)
# Motor 4
GPIO.setup(MOTOR4_IN1, GPIO.OUT)
GPIO.setup(MOTOR4_IN2, GPIO.OUT)
GPIO.setup(MOTOR4_SPD, GPIO.OUT)

# Set up PWM for all motors
motor1_pwm = GPIO.PWM(MOTOR1_SPD, 1000)
motor2_pwm = GPIO.PWM(MOTOR2_SPD, 1000)
motor3_pwm = GPIO.PWM(MOTOR3_SPD, 1000)
motor4_pwm = GPIO.PWM(MOTOR4_SPD, 1000)

# Start PWM for all motors
motor1_pwm.start(0)
motor2_pwm.start(0)
motor3_pwm.start(0)
motor4_pwm.start(0)

# Set up MCP3008
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

class SpikeFilter:
    def __init__(self, name, flip_direction=False):
        self.filter_active = False
        self.last_valid_reading = None
        self.name = name
        self.flip_direction = flip_direction

    def filter(self, new_value):
        # If the filter is active, we are in the dead zone
        if self.filter_active:
            if not self.flip_direction:
                # Normal orientation: Discard readings between 150 and 700
                if 150 <= new_value <= 700:
                    print(f"[{self.name}] Discarding invalid reading during dead zone: {new_value}")
                    return None
            else:
                # Flipped orientation: Discard readings between 300 and 850
                if 300 <= new_value <= 850:
                    print(f"[{self.name}] Discarding invalid reading during dead zone: {new_value}")
                    return None
                
            # Valid reading after dead zone
            print(f"[{self.name}] Valid reading after dead zone: {new_value}")
            self.filter_active = False
            self.last_valid_reading = new_value
            return new_value
        else:
            # Not currently filtering
            if self.last_valid_reading is not None:
                if not self.flip_direction:
                    # Normal orientation: Check for sudden drop from high to middle
                    if self.last_valid_reading > 950 and 150 <= new_value <= 700:
                        print(f"[{self.name}] Dead zone detected: last valid {self.last_valid_reading}, new {new_value}")
                        self.filter_active = True
                        return None
                else:
                    # Flipped orientation: Check for sudden rise from low to middle
                    if self.last_valid_reading < 50 and 300 <= new_value <= 850:
                        print(f"[{self.name}] Dead zone detected: last valid {self.last_valid_reading}, new {new_value}")
                        self.filter_active = True
                        return None
                    
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
    def __init__(self, name, in1, in2, pwm, adc_channel, target_position, phase_shift=0, flip_direction=False):
        self.name = name
        self.in1 = in1
        self.in2 = in2
        self.pwm = pwm
        self.adc_channel = adc_channel
        self.target_position = target_position
        self.phase_shift = phase_shift
        self.flip_direction = flip_direction
        self.position = 0
        self.in_dead_zone = False
        self.last_valid_position = None
        self.pid = PIDController(Kp=0.8, Ki=0.1, Kd=0.05)
        self.start_time = None
        self.spike_filter = SpikeFilter(name, flip_direction)
        
    def read_position(self):
        # Read raw ADC value
        raw_value = mcp.read_adc(self.adc_channel)
        
        # Apply spike filter
        filtered_value = self.spike_filter.filter(raw_value)
        
        if filtered_value is None:
            # Use last valid position if in dead zone
            return self.last_valid_position if self.last_valid_position is not None else 0
        
        # Convert filtered ADC value to degrees
        degrees = (filtered_value / ADC_MAX) * 330.0
        
        # If the motor is flipped, invert the position reading
        if self.flip_direction:
            degrees = MAX_ANGLE - degrees
            
        self.last_valid_position = degrees
        return degrees

    def set_motor_direction(self, direction):
        # Flip the direction for motors on the opposite side
        if self.flip_direction:
            direction = 'backward' if direction == 'forward' else 'forward'
            
        if direction == 'forward':
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)

    def move_to_position(self, target):
        current_position = self.read_position()
        
        # If the motor is flipped, we need to adjust the target
        if self.flip_direction:
            target = MAX_ANGLE - target
            
        error = target - current_position

        # Normalize error for circular movement
        if error > 165:
            error -= 330
        elif error < -165:
            error += 330

        # Use PID to compute control signal
        control_signal = self.pid.compute(error)

        # Determine direction and speed
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
        
        # Apply phase shift
        shifted_position = (position_in_cycle * MAX_ANGLE + self.phase_shift) % MAX_ANGLE
        
        # Reset to 0 when reaching MAX_ANGLE
        if shifted_position >= MAX_ANGLE:
            shifted_position = 0
            
        return shifted_position

def main():
    parser = argparse.ArgumentParser(description="Motor control with sawtooth pattern")
    parser.add_argument("initial_position", type=int, nargs="?", default=90,
                       help="Initial position for motors 1 and 4")
    parser.add_argument("shifted_position", type=int, nargs="?", default=270,
                       help="Initial position for motors 2 and 3")
    args = parser.parse_args()

    try:
        # Initialize motors with correct flip_direction for both motor control and encoder reading
        motor1 = MotorController("Motor 1", MOTOR1_IN1, MOTOR1_IN2, motor1_pwm, 
                               MOTOR1_ADC_CHANNEL, args.initial_position, phase_shift=0, flip_direction=False)
        motor3 = MotorController("Motor 3", MOTOR3_IN1, MOTOR3_IN2, motor3_pwm, 
                               MOTOR3_ADC_CHANNEL, args.shifted_position, phase_shift=PHASE_SHIFT, flip_direction=False)
        
        # Motors 2 and 4 are flipped in both direction and encoder reading
        motor2 = MotorController("Motor 2", MOTOR2_IN1, MOTOR2_IN2, motor2_pwm, 
                               MOTOR2_ADC_CHANNEL, args.shifted_position, phase_shift=PHASE_SHIFT, flip_direction=True)
        motor4 = MotorController("Motor 4", MOTOR4_IN1, MOTOR4_IN2, motor4_pwm, 
                               MOTOR4_ADC_CHANNEL, args.initial_position, phase_shift=0, flip_direction=True)

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