#!/usr/bin/env python3
import RPi.GPIO as GPIO
import spidev
import time
import logging

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# Constants
ADC_MAX = 1023
MAX_ANGLE = 360.0
SAWTOOTH_PERIOD = 5.0  # seconds

# GPIO Pin Definitions (Adjust these based on your wiring)
MOTOR1_IN1 = 17
MOTOR1_IN2 = 27
MOTOR1_PWM = 22

MOTOR2_IN1 = 23
MOTOR2_IN2 = 24
MOTOR2_PWM = 25

MOTOR3_IN1 = 5
MOTOR3_IN2 = 6
MOTOR3_PWM = 12

MOTOR4_IN1 = 13
MOTOR4_IN2 = 19
MOTOR4_PWM = 26

# ADC Channel Definitions (Assuming MCP3008)
MOTOR1_ADC_CHANNEL = 0
MOTOR2_ADC_CHANNEL = 1
MOTOR3_ADC_CHANNEL = 2
MOTOR4_ADC_CHANNEL = 3

# PID Constants (Kp, Ki, Kd) for each motor
PID_CONSTANTS = {
    "Motor 1": (0.8, 0.1, 0.05),
    "Motor 2": (0.5, 0.05, 0.02),
    "Motor 3": (0.8, 0.1, 0.05),
    "Motor 4": (0.5, 0.05, 0.02)
}

# Target Positions (Degrees)
INITIAL_POSITION = 90.0
SHIFTED_POSITION = 270.0

# Dead Zone Thresholds (ADC Values)
DEAD_ZONE_MIN = 150
DEAD_ZONE_MAX = 700

class SpikeFilter:
    def __init__(self, name, flip_filter=False):
        self.filter_active = False
        self.last_valid_reading = None
        self.name = name
        self.flip_filter = flip_filter

    def filter(self, new_value):
        if self.filter_active:
            if self.flip_filter:
                if new_value >= DEAD_ZONE_MAX or new_value <= DEAD_ZONE_MIN:
                    logging.debug(f"[{self.name}] Discarding invalid reading during dead zone: {new_value}")
                    return None
            else:
                if DEAD_ZONE_MIN <= new_value <= DEAD_ZONE_MAX:
                    logging.debug(f"[{self.name}] Discarding invalid reading during dead zone: {new_value}")
                    return None
            # Valid reading after dead zone
            logging.debug(f"[{self.name}] Valid reading after dead zone: {new_value}")
            self.filter_active = False
            self.last_valid_reading = new_value
            return new_value
        else:
            if self.last_valid_reading is not None:
                if self.flip_filter:
                    if new_value < DEAD_ZONE_MIN or new_value > DEAD_ZONE_MAX:
                        # Spike detected
                        logging.debug(f"[{self.name}] Spike detected: last valid {self.last_valid_reading}, new {new_value}")
                        self.filter_active = True
                        return None
                else:
                    if DEAD_ZONE_MIN <= new_value <= DEAD_ZONE_MAX:
                        # Dead zone detected
                        logging.debug(f"[{self.name}] Dead zone detected: last valid {self.last_valid_reading}, new {new_value}")
                        self.filter_active = True
                        return None
            # Valid reading
            self.last_valid_reading = new_value
            return new_value

class PIDController:
    def __init__(self, Kp, Ki, Kd, name="PID", output_limits=(None, None)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.name = name
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()
        self.min_output, self.max_output = output_limits

    def compute(self, error):
        current_time = time.time()
        delta_time = current_time - self.last_time
        if delta_time <= 0.0:
            delta_time = 0.0001  # Prevent division by zero

        proportional = self.Kp * error
        self.integral += error * delta_time
        integral = self.Ki * self.integral
        derivative = self.Kd * (error - self.previous_error) / delta_time

        control_signal = proportional + integral + derivative

        # Clamp control signal
        if self.min_output is not None:
            control_signal = max(self.min_output, control_signal)
        if self.max_output is not None:
            control_signal = min(self.max_output, control_signal)

        self.previous_error = error
        self.last_time = current_time

        logging.debug(f"[{self.name}] P: {proportional:.2f}, I: {integral:.2f}, D: {derivative:.2f}, Control: {control_signal:.2f}")
        return control_signal

class MotorController:
    def __init__(self, name, in1, in2, pwm_pin, adc_channel, target_position, phase_shift=0, flip_direction=False, flip_filter=False, pid_constants=(0.8, 0.1, 0.05)):
        self.name = name
        self.in1 = in1
        self.in2 = in2
        self.pwm_pin = pwm_pin
        self.adc_channel = adc_channel
        self.target_position = target_position
        self.phase_shift = phase_shift
        self.flip_direction = flip_direction
        self.position = 0
        self.spike_filter = SpikeFilter(name, flip_filter=flip_filter)
        self.pid = PIDController(Kp=pid_constants[0], Ki=pid_constants[1], Kd=pid_constants[2], name=f"{self.name} PID", output_limits=(-100, 100))
        self.pwm = None

    def initialize(self):
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pwm_pin, 1000)  # 1kHz frequency
        self.pwm.start(0)

    def read_position(self):
        raw_value = read_adc(self.adc_channel)
        logging.debug(f"[{self.name}] Raw ADC Value: {raw_value}")
        filtered_value = self.spike_filter.filter(raw_value)
        logging.debug(f"[{self.name}] Filtered ADC Value: {filtered_value}")

        if filtered_value is None:
            return self.position  # Return last known position

        degrees = (filtered_value / ADC_MAX) * MAX_ANGLE

        if self.flip_direction:
            degrees = MAX_ANGLE - degrees
            logging.debug(f"[{self.name}] Flipped Position: {degrees:.2f}°")
        else:
            logging.debug(f"[{self.name}] Position: {degrees:.2f}°")

        self.position = degrees
        return degrees

    def set_motor_direction(self, direction):
        if self.flip_direction:
            direction = 'backward' if direction == 'forward' else 'forward'
            logging.debug(f"[{self.name}] Adjusted direction due to flip: {direction}")

        if direction == 'forward':
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)

    def move_to_position(self, target):
        current_position = self.read_position()

        # Calculate error
        error = target - current_position

        # Normalize error for circular movement
        if error > (MAX_ANGLE / 2):
            error -= MAX_ANGLE
        elif error < -(MAX_ANGLE / 2):
            error += MAX_ANGLE

        # Invert error for flipped motors
        if self.flip_direction:
            error = -error
            logging.debug(f"[{self.name}] Inverted Error due to flip: {error:.2f}°")

        logging.debug(f"[{self.name}] Target: {target:.2f}°, Current: {current_position:.2f}°, Error: {error:.2f}°")

        # Use PID to compute control signal
        control_signal = self.pid.compute(error)

        # Clamp control signal
        control_signal = max(-100, min(100, control_signal))

        # Determine if target is reached within tolerance
        if abs(error) <= 2.0:  # 2 degrees tolerance
            self.stop_motor()
            logging.info(f"[{self.name}] Target reached within tolerance.")
            return True

        # Determine direction based on control signal
        direction = 'forward' if control_signal > 0 else 'backward'
        self.set_motor_direction(direction)

        # Adjust speed scaling
        speed = max(30, min(100, abs(control_signal)))
        logging.debug(f"[{self.name}] Control Signal: {control_signal:.2f}, Speed: {speed}%")
        self.pwm.ChangeDutyCycle(speed)
        return False

    def stop_motor(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
        logging.debug(f"[{self.name}] Motor stopped.")

    def cleanup(self):
        self.pwm.stop()
        self.stop_motor()

def read_adc(channel):
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1]&3) << 8) + adc[2]
    return data

def initialize_spi():
    spi.open(0, 0)  # SPI bus 0, device (CS) 0
    spi.max_speed_hz = 1350000

def cleanup_gpio(motors):
    for motor in motors:
        motor.cleanup()
    GPIO.cleanup()
    spi.close()
    logging.info("GPIO cleaned up and SPI closed.")

def main():
    global spi
    spi = spidev.SpiDev()

    # Initialize GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Initialize SPI for ADC
    initialize_spi()

    # Initialize Motors
    motor1 = MotorController(
        name="Motor 1",
        in1=MOTOR1_IN1,
        in2=MOTOR1_IN2,
        pwm_pin=MOTOR1_PWM,
        adc_channel=MOTOR1_ADC_CHANNEL,
        target_position=INITIAL_POSITION,
        flip_direction=False,
        flip_filter=False,
        pid_constants=PID_CONSTANTS["Motor 1"]
    )
    motor2 = MotorController(
        name="Motor 2",
        in1=MOTOR2_IN1,
        in2=MOTOR2_IN2,
        pwm_pin=MOTOR2_PWM,
        adc_channel=MOTOR2_ADC_CHANNEL,
        target_position=SHIFTED_POSITION,
        flip_direction=True,
        flip_filter=True,
        pid_constants=PID_CONSTANTS["Motor 2"]
    )
    motor3 = MotorController(
        name="Motor 3",
        in1=MOTOR3_IN1,
        in2=MOTOR3_IN2,
        pwm_pin=MOTOR3_PWM,
        adc_channel=MOTOR3_ADC_CHANNEL,
        target_position=SHIFTED_POSITION,
        flip_direction=False,
        flip_filter=False,
        pid_constants=PID_CONSTANTS["Motor 3"]
    )
    motor4 = MotorController(
        name="Motor 4",
        in1=MOTOR4_IN1,
        in2=MOTOR4_IN2,
        pwm_pin=MOTOR4_PWM,
        adc_channel=MOTOR4_ADC_CHANNEL,
        target_position=INITIAL_POSITION,
        flip_direction=True,
        flip_filter=True,
        pid_constants=PID_CONSTANTS["Motor 4"]
    )

    motors = [motor1, motor2, motor3, motor4]

    try:
        # Initialize all motors
        for motor in motors:
            motor.initialize()

        # Calibration parameters
        calibration_timeout = 30  # seconds
        calibration_start = time.time()

        logging.info("Starting calibration...")

        while True:
            # Attempt to move each motor to its target position
            m1_done = motor1.move_to_position(INITIAL_POSITION)
            m2_done = motor2.move_to_position(SHIFTED_POSITION)
            m3_done = motor3.move_to_position(SHIFTED_POSITION)
            m4_done = motor4.move_to_position(INITIAL_POSITION)

            # Display calibration progress
            logging.info(f"Calibrating - "
                         f"M1: {motor1.position:.1f}° / {INITIAL_POSITION}°, "
                         f"M2: {motor2.position:.1f}° / {SHIFTED_POSITION}°, "
                         f"M3: {motor3.position:.1f}° / {SHIFTED_POSITION}°, "
                         f"M4: {motor4.position:.1f}° / {INITIAL_POSITION}°")

            # Check if all motors have reached their targets
            if m1_done and m2_done and m3_done and m4_done:
                logging.info("Calibration complete.")
                break

            # Check for calibration timeout
            if time.time() - calibration_start > calibration_timeout:
                logging.error("Calibration timeout! Check motor connections.")
                raise TimeoutError("Calibration timed out.")

            time.sleep(0.05)  # 50ms delay between iterations

        # Optional: Hold motors at target positions for a moment
        logging.info("Holding positions for 5 seconds...")
        time.sleep(5)

    except TimeoutError as te:
        logging.error(str(te))
    except KeyboardInterrupt:
        logging.info("Program interrupted by user.")
    finally:
        # Cleanup GPIO and SPI
        cleanup_gpio(motors)

if __name__ == "__main__":
    main()
