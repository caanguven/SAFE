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
SAWTOOTH_PERIOD = 5  # Period in seconds
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
        self.pid = PIDController(Kp=0.7, Ki=0.1, Kd=0.05)
        self.start_time = None
        
    def read_position(self):
        adc_value = mcp.read_adc(self.adc_channel)
        if adc_value < DEAD_ZONE_THRESHOLD or adc_value > (ADC_MAX - DEAD_ZONE_THRESHOLD):
            return self.last_valid_position if self.last_valid_position is not None else 0
        
        degrees = (adc_value / ADC_MAX) * 330.0
        self.last_valid_position = degrees
        return degrees

    def set_motor_direction(self, direction):
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
    parser.add_argument("motor1_target", type=int, nargs="?", default=90)
    parser.add_argument("motor3_target", type=int, nargs="?", default=270)
    args = parser.parse_args()

    try:
        # Initialize motors with phase shift for Motor 3
        motor1 = MotorController("Motor 1", MOTOR1_IN1, MOTOR1_IN2, motor1_pwm, 
                               MOTOR1_ADC_CHANNEL, args.motor1_target, phase_shift=0)
        motor3 = MotorController("Motor 3", MOTOR3_IN1, MOTOR3_IN2, motor3_pwm, 
                               MOTOR3_ADC_CHANNEL, args.motor3_target, phase_shift=PHASE_SHIFT)

        # Initial calibration
        print("Starting initial calibration...")
        while True:
            m1_done = motor1.move_to_position(args.motor1_target)
            m3_done = motor3.move_to_position(args.motor3_target)
            
            if m1_done and m3_done:
                break
            time.sleep(0.05)

        print("Calibration complete. Stopping for 5 seconds...")
        motor1.stop_motor()
        motor3.stop_motor()
        time.sleep(5)

        print("Starting sawtooth pattern with 180-degree phase shift for Motor 3...")
        while True:
            # Generate sawtooth positions
            m1_target = motor1.generate_sawtooth_position()
            m3_target = motor3.generate_sawtooth_position()
            
            # Move motors to positions
            motor1.move_to_position(m1_target)
            motor3.move_to_position(m3_target)
            
            # Print current positions
            m1_pos = motor1.read_position()
            m3_pos = motor3.read_position()
            print(f"Motor 1 - Target: {m1_target:.1f}°, Current: {m1_pos:.1f}°")
            print(f"Motor 3 - Target: {m3_target:.1f}°, Current: {m3_pos:.1f}°")
            print(f"Phase Difference: {abs(m3_pos - m1_pos):.1f}°")
            
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    finally:
        motor1.stop_motor()
        motor3.stop_motor()
        motor1_pwm.stop()
        motor3_pwm.stop()
        GPIO.cleanup()
        print("GPIO cleaned up")

if __name__ == "__main__":
    main()