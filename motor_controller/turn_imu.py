import warnings
# Suppress specific I2C frequency warning
warnings.filterwarnings("ignore", category=RuntimeWarning, module="adafruit_blinka.microcontroller.generic_linux.i2c")

import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import busio
import board
import adafruit_bno08x  # Import the module for accessing constants
from adafruit_bno08x.i2c import BNO08X_I2C
import curses
import sys
import logging
import math
import signal

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("turn_based_on_yaw.log"),
        logging.StreamHandler()
    ]
)

# Suppress GPIO warnings
GPIO.setwarnings(False)

# Function to handle SIGINT (Ctrl+C) gracefully
def signal_handler(sig, frame):
    logging.info("Interrupt received. Cleaning up GPIO and exiting.")
    stop_all_motors()
    for pwm in motor_pwms.values():
        pwm.stop()
    GPIO.cleanup()
    curses.endwin()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Constants for SPI and ADC
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023
MIN_ANGLE = 0
MAX_ANGLE = 330
SAWTOOTH_PERIOD = 2  # Period in seconds

# GPIO Pins for Motor Control (Using BCM numbering)
MOTOR1_IN1 = 4    # BCM Pin 4 (Originally BOARD pin 7)
MOTOR1_IN2 = 7    # BCM Pin 7 (Originally BOARD pin 26)
MOTOR1_SPD = 24   # BCM Pin 24 (Originally BOARD pin 18)
MOTOR1_ADC_CHANNEL = 0

MOTOR2_IN1 = 5    # BCM Pin 5 (Originally BOARD pin 29)
MOTOR2_IN2 = 25   # BCM Pin 25 (Originally BOARD pin 22)
MOTOR2_SPD = 6    # BCM Pin 6 (Originally BOARD pin 31)
MOTOR2_ADC_CHANNEL = 1

MOTOR3_IN1 = 17   # BCM Pin 17 (Originally BOARD pin 11)
MOTOR3_IN2 = 12   # BCM Pin 12 (Originally BOARD pin 32)
MOTOR3_SPD = 13   # BCM Pin 13 (Originally BOARD pin 33)
MOTOR3_ADC_CHANNEL = 2

MOTOR4_IN1 = 18   # BCM Pin 18 (Originally BOARD pin 12)
MOTOR4_IN2 = 27   # BCM Pin 27 (Originally BOARD pin 13)
MOTOR4_SPD = 19   # BCM Pin 19 (Originally BOARD pin 35)
MOTOR4_ADC_CHANNEL = 3

# Function to set GPIO mode safely
def safe_setmode(mode):
    current_mode = GPIO.getmode()
    if current_mode is None:
        GPIO.setmode(mode)
        logging.info(f"GPIO mode set to {mode}.")
    elif current_mode == mode:
        logging.info(f"GPIO mode already set to {mode}.")
    else:
        logging.warning(f"GPIO mode already set to {current_mode}. Cleaning up and resetting to {mode}.")
        GPIO.cleanup()
        try:
            GPIO.setmode(mode)
            logging.info(f"GPIO mode reset to {mode}.")
        except ValueError as ve:
            logging.error(f"Failed to set GPIO mode to {mode}: {ve}")
            sys.exit(1)

# Set GPIO mode safely to BCM
safe_setmode(GPIO.BCM)

# Setup GPIO pins
motor_pins = [
    (MOTOR1_IN1, MOTOR1_IN2, MOTOR1_SPD),
    (MOTOR2_IN1, MOTOR2_IN2, MOTOR2_SPD),
    (MOTOR3_IN1, MOTOR3_IN2, MOTOR3_SPD),
    (MOTOR4_IN1, MOTOR4_IN2, MOTOR4_SPD)
]

for in1, in2, spd in motor_pins:
    try:
        GPIO.setup(in1, GPIO.OUT)
        GPIO.setup(in2, GPIO.OUT)
        GPIO.setup(spd, GPIO.OUT)
        logging.debug(f"Set up GPIO pins: IN1={in1}, IN2={in2}, SPD={spd}")
    except Exception as e:
        logging.exception(f"Error setting up GPIO pins {in1}, {in2}, {spd}:")
        GPIO.cleanup()
        sys.exit(1)

# Initialize PWM for all motors
motor_pwms = {}
for i, (in1, in2, spd) in enumerate(motor_pins, start=1):
    try:
        pwm = GPIO.PWM(spd, 1000)  # 1kHz frequency
        pwm.start(0)
        motor_pwms[f"M{i}"] = pwm
        logging.debug(f"Initialized PWM for Motor M{i} on pin {spd}.")
    except Exception as e:
        logging.exception(f"Error initializing PWM for Motor M{i}:")
        GPIO.cleanup()
        sys.exit(1)

# Initialize MCP3008
try:
    mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))
    logging.info("MCP3008 ADC initialized.")
except Exception as e:
    logging.exception("Error initializing MCP3008:")
    GPIO.cleanup()
    sys.exit(1)

# Initialize IMU
def quaternion_to_euler(quat_i, quat_j, quat_k, quat_real):
    roll = math.atan2(2 * (quat_real * quat_i + quat_j * quat_k),
                     1 - 2 * (quat_i**2 + quat_j**2))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (quat_real * quat_j - quat_k * quat_i))))
    yaw = math.atan2(2 * (quat_real * quat_k + quat_i * quat_j),
                    1 - 2 * (quat_j**2 + quat_k**2))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def setup_imu():
    try:
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        bno = BNO08X_I2C(i2c)
        bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)  # Corrected attribute
        logging.info("IMU initialized and rotation vector reporting enabled.")
        return bno
    except Exception as e:
        logging.exception("Failed to initialize IMU:")
        GPIO.cleanup()
        sys.exit(1)

bno = setup_imu()

def calibrate_imu(bno):
    """
    Capture the initial IMU readings to use as calibration offsets.
    Assumes the robot starts in a calibrated state.
    """
    try:
        logging.info("Calibrating IMU (capturing initial orientation)...")
        quat = bno.quaternion
        if quat is not None:
            quat_i, quat_j, quat_k, quat_real = quat
            roll, pitch, yaw = quaternion_to_euler(quat_i, quat_j, quat_k, quat_real)
            calibration_offset = yaw
            logging.info(f"IMU calibrated. Calibration Yaw Offset: {calibration_offset:.2f}°")
            return calibration_offset
        else:
            logging.error("No quaternion data available for calibration.")
            GPIO.cleanup()
            sys.exit(1)
    except Exception as e:
        logging.exception("Error during IMU calibration:")
        GPIO.cleanup()
        sys.exit(1)

# Perform manual calibration
calibration_offset = calibrate_imu(bno)

def get_current_yaw(bno, calibration_offset):
    try:
        quat = bno.quaternion
        if quat is not None:
            _, _, yaw = quaternion_to_euler(*quat)
            adjusted_yaw = (yaw - calibration_offset) % 360
            logging.debug(f"Current Yaw: {adjusted_yaw:.2f}°")
            return adjusted_yaw
        else:
            logging.warning("No quaternion data available.")
            return None
    except Exception as e:
        logging.exception("Error getting current yaw:")
        return None

# Define motor control functions
def set_motor_direction(motor, direction):
    in1, in2, _ = motor_pins[int(motor[1])-1]
    if direction == 'forward':
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    elif direction == 'backward':
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
    else:  # stop
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
    logging.debug(f"Motor {motor} direction set to {direction}.")

def set_motor_speed(motor, speed):
    motor_pwms[motor].ChangeDutyCycle(speed)
    logging.debug(f"Motor {motor} speed set to {speed}%.")

def stop_all_motors():
    for motor in motor_pwms:
        set_motor_direction(motor, 'stop')
        set_motor_speed(motor, 0)
    logging.info("All motors stopped.")

# Initialize curses for keyboard input
def main(stdscr):
    logging.info("Curses main function initiated.")

    curses.cbreak()
    curses.noecho()
    stdscr.nodelay(True)
    stdscr.keypad(True)

    stdscr.addstr(0, 0, "Yaw-Based Turning Control")
    stdscr.addstr(2, 0, "Controls:")
    stdscr.addstr(3, 2, "Left Arrow  : Turn Left")
    stdscr.addstr(4, 2, "Right Arrow : Turn Right")
    stdscr.addstr(5, 2, "Space       : Stop")
    stdscr.addstr(6, 2, "Q           : Quit")
    stdscr.refresh()

    target_turn_angle = 30  # degrees
    turning = False
    turn_direction = None
    initial_yaw = None
    target_yaw = None

    try:
        while True:
            key = stdscr.getch()

            if key != -1:
                if key == curses.KEY_LEFT:
                    if not turning:
                        turning = True
                        turn_direction = 'left'
                        initial_yaw = get_current_yaw(bno, calibration_offset)
                        if initial_yaw is not None:
                            target_yaw = (initial_yaw - target_turn_angle) % 360
                            # To turn left: Left motors backward, Right motors forward
                            set_motor_direction('M1', 'backward')  # Left Front
                            set_motor_direction('M2', 'forward')   # Right Front
                            set_motor_direction('M3', 'backward')  # Left Rear
                            set_motor_direction('M4', 'forward')   # Right Rear
                            set_motor_speed('M1', 50)
                            set_motor_speed('M2', 50)
                            set_motor_speed('M3', 50)
                            set_motor_speed('M4', 50)
                            logging.info(f"Initiating left turn to {target_yaw}° from {initial_yaw}°")
                elif key == curses.KEY_RIGHT:
                    if not turning:
                        turning = True
                        turn_direction = 'right'
                        initial_yaw = get_current_yaw(bno, calibration_offset)
                        if initial_yaw is not None:
                            target_yaw = (initial_yaw + target_turn_angle) % 360
                            # To turn right: Left motors forward, Right motors backward
                            set_motor_direction('M1', 'forward')   # Left Front
                            set_motor_direction('M2', 'backward')  # Right Front
                            set_motor_direction('M3', 'forward')   # Left Rear
                            set_motor_direction('M4', 'backward')  # Right Rear
                            set_motor_speed('M1', 50)
                            set_motor_speed('M2', 50)
                            set_motor_speed('M3', 50)
                            set_motor_speed('M4', 50)
                            logging.info(f"Initiating right turn to {target_yaw}° from {initial_yaw}°")
                elif key == ord(' '):
                    turning = False
                    turn_direction = None
                    stop_all_motors()
                    logging.info("Manual stop command received.")
                elif key in [ord('q'), ord('Q')]:
                    logging.info("Quit command received.")
                    break

            if turning and initial_yaw is not None and target_yaw is not None:
                current_yaw = get_current_yaw(bno, calibration_offset)
                if current_yaw is not None:
                    if turn_direction == 'left':
                        # Calculate the smallest angular difference
                        diff = (initial_yaw - current_yaw) % 360
                        if diff >= target_turn_angle:
                            turning = False
                            stop_all_motors()
                            logging.info(f"Left turn completed. Current Yaw: {current_yaw}°")
                    elif turn_direction == 'right':
                        diff = (current_yaw - initial_yaw) % 360
                        if diff >= target_turn_angle:
                            turning = False
                            stop_all_motors()
                            logging.info(f"Right turn completed. Current Yaw: {current_yaw}°")

            # Update display
            yaw = get_current_yaw(bno, calibration_offset)
            if yaw is not None:
                stdscr.addstr(8, 0, f"Current Yaw: {yaw:.2f}°")
            else:
                stdscr.addstr(8, 0, "Current Yaw: N/A")
            stdscr.addstr(10, 0, f"Turning: {'Yes' if turning else 'No'} Direction: {turn_direction if turn_direction else 'N/A'}")
            stdscr.refresh()
            time.sleep(0.1)

    except Exception as e:
        logging.exception("An error occurred in the main loop:")
    finally:
        logging.info("Entering cleanup phase.")
        stop_all_motors()
        for pwm in motor_pwms.values():
            pwm.stop()
        GPIO.cleanup()
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()
        logging.info("GPIO cleanup completed.")

if __name__ == "__main__":
    try:
        logging.info("Script started.")
        curses.wrapper(main)
    except Exception as e:
        logging.exception("An unexpected error occurred:")
        stop_all_motors()
        for pwm in motor_pwms.values():
            pwm.stop()
        GPIO.cleanup()
        sys.exit(1)
