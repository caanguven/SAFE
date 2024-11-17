import warnings
# Suppress specific I2C frequency warning
warnings.filterwarnings("ignore", category=RuntimeWarning, module="adafruit_blinka.microcontroller.generic_linux.i2c")

import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import busio
import board
import adafruit_bno08x
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

# Constants for SPI and ADC
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023
MIN_ANGLE = 0
MAX_ANGLE = 330

# GPIO Pins for Motor Control (Using BCM numbering)
MOTOR1_IN1 = 4    # Left Front
MOTOR1_IN2 = 7
MOTOR1_SPD = 24

MOTOR2_IN1 = 5    # Right Front
MOTOR2_IN2 = 25
MOTOR2_SPD = 6

MOTOR3_IN1 = 17   # Left Rear
MOTOR3_IN2 = 12
MOTOR3_SPD = 13

MOTOR4_IN1 = 18   # Right Rear
MOTOR4_IN2 = 27
MOTOR4_SPD = 19

# Function to handle SIGINT (Ctrl+C) gracefully
def signal_handler(sig, frame):
    print("Interrupt received. Cleaning up GPIO and exiting.")
    logging.info("Interrupt received. Cleaning up GPIO and exiting.")
    stop_all_motors()
    for pwm in motor_pwms.values():
        pwm.stop()
    GPIO.cleanup()
    curses.endwin()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def safe_setmode(mode):
    current_mode = GPIO.getmode()
    if current_mode is None:
        GPIO.setmode(mode)
        logging.info(f"GPIO mode set to {mode}")
    elif current_mode != mode:
        GPIO.cleanup()
        GPIO.setmode(mode)
        logging.info(f"GPIO mode reset to {mode}")

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
    GPIO.setup(in1, GPIO.OUT)
    GPIO.setup(in2, GPIO.OUT)
    GPIO.setup(spd, GPIO.OUT)

# Initialize PWM for all motors with 100Hz frequency
motor_pwms = {}
for i, (in1, in2, spd) in enumerate(motor_pins, start=1):
    pwm = GPIO.PWM(spd, 100)  # Changed to 100Hz
    pwm.start(0)
    motor_pwms[f"M{i}"] = pwm

def quaternion_to_euler(quat_i, quat_j, quat_k, quat_real):
    """Convert quaternion to Euler angles (roll, pitch, yaw)"""
    roll = math.atan2(2 * (quat_real * quat_i + quat_j * quat_k),
                     1 - 2 * (quat_i**2 + quat_j**2))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (quat_real * quat_j - quat_k * quat_i))))
    yaw = math.atan2(2 * (quat_real * quat_k + quat_i * quat_j),
                    1 - 2 * (quat_j**2 + quat_k**2))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def get_shortest_angle(angle1, angle2):
    """Calculate shortest angular distance between two angles"""
    diff = (angle2 - angle1) % 360
    if diff > 180:
        diff -= 360
    return diff

def verify_imu_data(bno):
    """Verify IMU is providing valid data"""
    try:
        quat = bno.quaternion
        if quat is None:
            logging.error("IMU not providing quaternion data")
            return False
        
        if any(math.isnan(x) for x in quat) or any(abs(x) > 1.0 for x in quat):
            logging.error("IMU providing invalid quaternion values")
            return False
            
        return True
    except Exception as e:
        logging.error(f"IMU verification failed: {e}")
        return False

def setup_imu():
    """Initialize and configure the IMU"""
    try:
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        bno = BNO08X_I2C(i2c)
        time.sleep(1)  # Startup delay
        
        bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
        time.sleep(1)  # Feature enable delay
        
        if not verify_imu_data(bno):
            raise RuntimeError("IMU initialization failed - not providing valid data")
            
        logging.info("IMU initialized successfully")
        return bno
    except Exception as e:
        logging.exception("Failed to initialize IMU:")
        GPIO.cleanup()
        sys.exit(1)

def calibrate_imu(bno):
    """Calibrate IMU with multiple samples"""
    try:
        logging.info("Calibrating IMU...")
        samples = []
        for _ in range(10):
            quat = bno.quaternion
            if quat is not None:
                _, _, yaw = quaternion_to_euler(*quat)
                samples.append(yaw)
            time.sleep(0.1)
            
        if samples:
            calibration_offset = sum(samples) / len(samples)
            logging.info(f"IMU calibrated. Average Yaw Offset: {calibration_offset:.2f}°")
            return calibration_offset
        else:
            raise RuntimeError("Calibration failed - no valid samples")
    except Exception as e:
        logging.exception("Error during IMU calibration:")
        GPIO.cleanup()
        sys.exit(1)

def get_current_yaw(bno, calibration_offset):
    """Get current yaw angle from IMU"""
    try:
        quat = bno.quaternion
        if quat is not None:
            _, _, yaw = quaternion_to_euler(*quat)
            adjusted_yaw = (yaw - calibration_offset) % 360
            logging.debug(f"Current Yaw: {adjusted_yaw:.2f}°")
            return adjusted_yaw
        return None
    except Exception as e:
        logging.exception("Error getting current yaw:")
        return None

def set_motor_direction(motor, direction):
    """Set motor direction (forward/backward/stop)"""
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
    logging.debug(f"Motor {motor} direction set to {direction}")

def set_motor_speed(motor, speed):
    """Set motor speed (0-100)"""
    motor_pwms[motor].ChangeDutyCycle(speed)
    logging.debug(f"Motor {motor} speed set to {speed}%")

def stop_all_motors():
    """Stop all motors"""
    for motor in motor_pwms:
        set_motor_direction(motor, 'stop')
        set_motor_speed(motor, 0)
    logging.info("All motors stopped")

def main(stdscr):
    """Main function for the motor control program"""
    logging.info("Starting main program")
    
    # Initialize IMU
    bno = setup_imu()
    calibration_offset = calibrate_imu(bno)
    
    # Setup curses
    curses.cbreak()
    curses.noecho()
    stdscr.nodelay(True)
    stdscr.keypad(True)

    # Display controls
    stdscr.addstr(0, 0, "IMU-Based Motor Control")
    stdscr.addstr(2, 0, "Controls:")
    stdscr.addstr(3, 2, "Left Arrow  : Turn Left")
    stdscr.addstr(4, 2, "Right Arrow : Turn Right")
    stdscr.addstr(5, 2, "Space       : Stop")
    stdscr.addstr(6, 2, "Q           : Quit")
    stdscr.refresh()

    target_turn_angle = 30  # degrees
    motor_speed = 70  # Increased from 50 to 70
    turning = False
    turn_direction = None
    initial_yaw = None

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
                            # Left turn configuration
                            set_motor_direction('M1', 'backward')
                            set_motor_direction('M2', 'forward')
                            set_motor_direction('M3', 'backward')
                            set_motor_direction('M4', 'forward')
                            for motor in motor_pwms:
                                set_motor_speed(motor, motor_speed)
                            logging.info(f"Starting left turn from {initial_yaw}°")
                
                elif key == curses.KEY_RIGHT:
                    if not turning:
                        turning = True
                        turn_direction = 'right'
                        initial_yaw = get_current_yaw(bno, calibration_offset)
                        if initial_yaw is not None:
                            # Right turn configuration
                            set_motor_direction('M1', 'forward')
                            set_motor_direction('M2', 'backward')
                            set_motor_direction('M3', 'forward')
                            set_motor_direction('M4', 'backward')
                            for motor in motor_pwms:
                                set_motor_speed(motor, motor_speed)
                            logging.info(f"Starting right turn from {initial_yaw}°")
                
                elif key == ord(' '):
                    turning = False
                    turn_direction = None
                    stop_all_motors()
                    logging.info("Manual stop")
                
                elif key in [ord('q'), ord('Q')]:
                    break

            if turning and initial_yaw is not None:
                if not verify_imu_data(bno):
                    turning = False
                    stop_all_motors()
                    logging.error("Stopping due to invalid IMU data")
                    continue

                current_yaw = get_current_yaw(bno, calibration_offset)
                if current_yaw is not None:
                    if turn_direction == 'left':
                        angle_diff = get_shortest_angle(current_yaw, initial_yaw)
                        if abs(angle_diff) >= target_turn_angle:
                            turning = False
                            stop_all_motors()
                            logging.info(f"Left turn completed. Final yaw: {current_yaw}°")
                    
                    elif turn_direction == 'right':
                        angle_diff = get_shortest_angle(initial_yaw, current_yaw)
                        if abs(angle_diff) >= target_turn_angle:
                            turning = False
                            stop_all_motors()
                            logging.info(f"Right turn completed. Final yaw: {current_yaw}°")

            # Update display
            current_yaw = get_current_yaw(bno, calibration_offset)
            if current_yaw is not None:
                stdscr.addstr(8, 0, f"Current Yaw: {current_yaw:.2f}°  ")
            stdscr.addstr(9, 0, f"Turning: {'Yes' if turning else 'No'}  ")
            stdscr.addstr(10, 0, f"Direction: {turn_direction if turn_direction else 'None'}  ")
            stdscr.refresh()
            
            time.sleep(0.05)  # Reduced sleep time for more responsive control

    except Exception as e:
        logging.exception("Error in main loop:")
    finally:
        stop_all_motors()
        for pwm in motor_pwms.values():
            pwm.stop()
        GPIO.cleanup()
        curses.endwin()

if __name__ == "__main__":
    try:
        curses.wrapper(main)
    except Exception as e:
        logging.exception("Fatal error:")
        stop_all_motors()
        for pwm in motor_pwms.values():
            pwm.stop()
        GPIO.cleanup()
        sys.exit(1)