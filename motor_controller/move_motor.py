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

# GPIO Pins for Motor 3
MOTOR3_IN1 = 11
MOTOR3_IN2 = 32
MOTOR3_SPD = 33
MOTOR3_ADC_CHANNEL = 2

# GPIO Pins for Motor 2 (flipped)
MOTOR2_IN1 = 13
MOTOR2_IN2 = 15
MOTOR2_SPD = 16
MOTOR2_ADC_CHANNEL = 1

# GPIO Pins for Motor 4 (flipped)
MOTOR4_IN1 = 19
MOTOR4_IN2 = 21
MOTOR4_SPD = 22
MOTOR4_ADC_CHANNEL = 3

# GPIO setup
GPIO.setmode(GPIO.BOARD)

# Setup for Motor 1
GPIO.setup(MOTOR1_IN1, GPIO.OUT)
GPIO.setup(MOTOR1_IN2, GPIO.OUT)
GPIO.setup(MOTOR1_SPD, GPIO.OUT)

# Setup for Motor 3
GPIO.setup(MOTOR3_IN1, GPIO.OUT)
GPIO.setup(MOTOR3_IN2, GPIO.OUT)
GPIO.setup(MOTOR3_SPD, GPIO.OUT)

# Setup for Motor 2
GPIO.setup(MOTOR2_IN1, GPIO.OUT)
GPIO.setup(MOTOR2_IN2, GPIO.OUT)
GPIO.setup(MOTOR2_SPD, GPIO.OUT)

# Setup for Motor 4
GPIO.setup(MOTOR4_IN1, GPIO.OUT)
GPIO.setup(MOTOR4_IN2, GPIO.OUT)
GPIO.setup(MOTOR4_SPD, GPIO.OUT)

# Set up PWM for motor speed control
motor1_pwm = GPIO.PWM(MOTOR1_SPD, 1000)
motor1_pwm.start(0)
motor3_pwm = GPIO.PWM(MOTOR3_SPD, 1000)
motor3_pwm.start(0)
motor2_pwm = GPIO.PWM(MOTOR2_SPD, 1000)
motor2_pwm.start(0)
motor4_pwm = GPIO.PWM(MOTOR4_SPD, 1000)
motor4_pwm.start(0)

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
    def __init__(self, name, in1, in2, pwm, adc_channel, target_position, phase_shift=0, flipped=False):
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
        self.flipped = flipped  # Indicates if encoder and direction are flipped

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
        if self.flipped:
            degrees = 330.0 - degrees  # Flip the encoder reading
        
        self.last_valid_position = degrees
        return degrees

    def set_motor_direction(self, direction):
        if self.flipped:
            # Invert direction
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
        
        # Apply phase shift
        shifted_position = (position_in_cycle * MAX_ANGLE + self.phase_shift) % MAX_ANGLE
        
        # Reset to 0 when reaching MAX_ANGLE
        if shifted_position >= MAX_ANGLE:
            shifted_position = 0
            
        return shifted_position

def main():
    parser = argparse.ArgumentParser(description="Motor control with sawtooth pattern for Quadruped Robot")
    parser.add_argument("--motor1_target", type=int, default=90, help="Initial target position for Motor 1")
    parser.add_argument("--motor3_target", type=int, default=270, help="Initial target position for Motor 3")
    parser.add_argument("--motor2_target", type=int, default=270, help="Initial target position for Motor 2")
    parser.add_argument("--motor4_target", type=int, default=90, help="Initial target position for Motor 4")
    args = parser.parse_args()

    try:
        # Initialize motors with phase shift and flipped settings
        motor1 = MotorController(
            name="Motor 1",
            in1=MOTOR1_IN1,
            in2=MOTOR1_IN2,
            pwm=motor1_pwm,
            adc_channel=MOTOR1_ADC_CHANNEL,
            target_position=args.motor1_target,
            phase_shift=0,
            flipped=False
        )
        motor3 = MotorController(
            name="Motor 3",
            in1=MOTOR3_IN1,
            in2=MOTOR3_IN2,
            pwm=motor3_pwm,
            adc_channel=MOTOR3_ADC_CHANNEL,
            target_position=args.motor3_target,
            phase_shift=PHASE_SHIFT,
            flipped=False
        )
        motor2 = MotorController(
            name="Motor 2",
            in1=MOTOR2_IN1,
            in2=MOTOR2_IN2,
            pwm=motor2_pwm,
            adc_channel=MOTOR2_ADC_CHANNEL,
            target_position=args.motor2_target,
            phase_shift=PHASE_SHIFT,
            flipped=True  # Encoder and direction are flipped
        )
        motor4 = MotorController(
            name="Motor 4",
            in1=MOTOR4_IN1,
            in2=MOTOR4_IN2,
            pwm=motor4_pwm,
            adc_channel=MOTOR4_ADC_CHANNEL,
            target_position=args.motor4_target,
            phase_shift=0,
            flipped=True  # Encoder and direction are flipped
        )

        motors = [motor1, motor2, motor3, motor4]

        # Initial calibration
        print("Starting initial calibration...")
        calibration_start = time.time()
        while True:
            calibration_done = True
            for motor in motors:
                done = motor.move_to_position(motor.target_position)
                if not done:
                    calibration_done = False

            # Print calibration progress
            positions = [f"{motor.name}: {motor.read_position():.1f}° / {motor.target_position}°" for motor in motors]
            print("\rCalibrating - " + ", ".join(positions), end='')

            if calibration_done:
                break

            # Timeout after 30 seconds
            if time.time() - calibration_start > 30:
                print("\nCalibration timeout! Check motor connections.")
                raise TimeoutError("Calibration timeout")
                
            time.sleep(0.05)

        print("\nCalibration complete. Stopping for 5 seconds...")
        for motor in motors:
            motor.stop_motor()
        time.sleep(5)

        print("Starting sawtooth pattern with phase shifts...")
        while True:
            # Generate sawtooth positions for each motor
            motor1_target_pos = motor1.generate_sawtooth_position()
            motor3_target_pos = motor3.generate_sawtooth_position()
            motor2_target_pos = motor2.generate_sawtooth_position()
            motor4_target_pos = motor4.generate_sawtooth_position()
            
            # Move motors to their target positions
            motor1.move_to_position(motor1_target_pos)
            motor3.move_to_position(motor3_target_pos)
            motor2.move_to_position(motor2_target_pos)
            motor4.move_to_position(motor4_target_pos)
            
            # Read current positions
            m1_pos = motor1.read_position()
            m2_pos = motor2.read_position()
            m3_pos = motor3.read_position()
            m4_pos = motor4.read_position()
            
            # Calculate phase differences if needed
            phase_diff_1_3 = abs(motor3_target_pos - motor1_target_pos)
            phase_diff_2_4 = abs(motor4_target_pos - motor2_target_pos)
            
            # Print current status
            print(f"\nMotor 1 - Target: {motor1_target_pos:.1f}°, Current: {m1_pos:.1f}°")
            print(f"Motor 3 - Target: {motor3_target_pos:.1f}°, Current: {m3_pos:.1f}°")
            print(f"Motor 2 - Target: {motor2_target_pos:.1f}°, Current: {m2_pos:.1f}°")
            print(f"Motor 4 - Target: {motor4_target_pos:.1f}°, Current: {m4_pos:.1f}°")
            print(f"Phase Difference Motor1-Motor3: {phase_diff_1_3:.1f}°, Motor2-Motor4: {phase_diff_2_4:.1f}°")
            
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
