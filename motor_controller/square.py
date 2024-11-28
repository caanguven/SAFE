import warnings
import argparse
import sys
import logging
import math
import time
import signal
import threading
import os
import traceback

import cv2
import numpy as np
from picamera2 import Picamera2
from pupil_apriltags import Detector

import RPi.GPIO as GPIO
import busio
import board
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
from datetime import datetime
import zipfile
import io

# Suppress specific warnings
warnings.filterwarnings("ignore", category=RuntimeWarning, module="adafruit_blinka.microcontroller.generic_linux.i2c")
warnings.filterwarnings("ignore", category=RuntimeWarning)

# Configure basic logging
logging.basicConfig(
    level=logging.INFO,  # Set to INFO to see informative messages
    format='%(message)s',
    handlers=[logging.StreamHandler()]
)

# Suppress GPIO warnings
GPIO.setwarnings(False)

# Constants for SPI and ADC
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023
MIN_ANGLE = 0
MAX_ANGLE = 330
SAWTOOTH_PERIOD = 2  # Period in seconds

# GPIO Pins configuration (using BCM numbering)
# Motor 1 (Left Front)
MOTOR1_IN1 = 4
MOTOR1_IN2 = 7
MOTOR1_SPD = 24
MOTOR1_ADC_CHANNEL = 0

# Motor 2 (Right Front)
MOTOR2_IN1 = 5
MOTOR2_IN2 = 25
MOTOR2_SPD = 6
MOTOR2_ADC_CHANNEL = 1

# Motor 3 (Left Rear)
MOTOR3_IN1 = 17
MOTOR3_IN2 = 12
MOTOR3_SPD = 13
MOTOR3_ADC_CHANNEL = 2

# Motor 4 (Right Rear)
MOTOR4_IN1 = 18
MOTOR4_IN2 = 27
MOTOR4_SPD = 19
MOTOR4_ADC_CHANNEL = 3

# AprilTag and Camera Parameters
FX = 1435.0  # Focal length in pixels along x-axis
FY = 1440.0  # Focal length in pixels along y-axis
CX = 1296.0  # Principal point x-coordinate
CY = 972.0   # Principal point y-coordinate
CAMERA_PARAMS = (FX, FY, CX, CY)
TAG_SIZE = 0.092  # Real size of the AprilTag (in meters)

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
            delta_time = 0.0001  # Prevent division by zero

        proportional = self.Kp * error
        self.integral += error * delta_time
        integral = self.Ki * self.integral
        derivative = self.Kd * (error - self.previous_error) / delta_time

        control_signal = proportional + integral + derivative
        self.previous_error = error
        self.last_time = current_time

        return control_signal

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
            if self.last_valid_reading is not None and abs(self.last_valid_reading - new_value) > 300:
                self.filter_active = True
                return None
            else:
                self.last_valid_reading = new_value
                return new_value

class MotorController:
    def __init__(self, name, in1, in2, pwm, adc_channel, encoder_flipped=False):
        self.name = name
        self.in1 = in1
        self.in2 = in2
        self.pwm = pwm
        self.adc_channel = adc_channel
        self.encoder_flipped = encoder_flipped
        self.position = 0
        self.last_valid_position = None
        self.pid = PIDController(Kp=0.8, Ki=0.1, Kd=0.05)
        self.spike_filter = SpikeFilter(name)

    def read_position(self, mcp):
        raw_value = mcp.read_adc(self.adc_channel)

        if self.encoder_flipped:
            raw_value = ADC_MAX - raw_value

        filtered_value = self.spike_filter.filter(raw_value)

        if filtered_value is None:
            return self.last_valid_position if self.last_valid_position is not None else 0

        degrees = (filtered_value / ADC_MAX) * 330.0
        self.last_valid_position = degrees
        return degrees

    def set_motor_direction(self, direction):
        if direction == 'forward':
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        elif direction == 'backward':
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
        else:  # stop
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.LOW)

    def set_motor_speed(self, speed):
        """Set the speed of the motor using PWM."""
        self.pwm.ChangeDutyCycle(speed)

    def move_to_position(self, target, mcp):
        current_position = self.read_position(mcp)
        error = target - current_position

        # Normalize error to range [-180, 180]
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        control_signal = self.pid.compute(error)

        if abs(error) <= 2:
            self.stop_motor()
            return True

        self.set_motor_direction('forward' if control_signal > 0 else 'backward')
        speed = min(100, max(30, abs(control_signal)))
        self.set_motor_speed(speed)
        return False

    def stop_motor(self):
        self.set_motor_direction('stop')
        self.set_motor_speed(0)

def quaternion_to_euler(quat_i, quat_j, quat_k, quat_real):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw)
    """
    try:
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (quat_real * quat_i + quat_j * quat_k)
        cosr_cosp = 1 - 2 * (quat_i * quat_i + quat_j * quat_j)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (quat_real * quat_j - quat_k * quat_i)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (quat_real * quat_k + quat_i * quat_j)
        cosy_cosp = 1 - 2 * (quat_j * quat_j + quat_k * quat_k)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Convert to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        return roll_deg, pitch_deg, yaw_deg

    except Exception as e:
        print(f"Error in quaternion conversion: {str(e)}")
        return None, None, None

def setup_motors():
    """Initialize GPIO pins and PWM for motors."""
    GPIO.setmode(GPIO.BCM)
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
    
    motor_pwms = {}
    for i, (in1, in2, spd) in enumerate(motor_pins, start=1):
        pwm = GPIO.PWM(spd, 100)  # 100 Hz frequency
        pwm.start(0)
        motor_pwms[f"M{i}"] = pwm
    
    return motor_pins, motor_pwms

def setup_imu():
    """Initialize IMU with retry mechanism"""
    max_retries = 3
    retry_delay = 2  # seconds
    
    for attempt in range(max_retries):
        try:
            print(f"Attempting to initialize IMU (attempt {attempt + 1}/{max_retries})...")
            
            i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
            time.sleep(0.5)
            
            bno = BNO08X_I2C(i2c)
            time.sleep(1)
            
            bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
            time.sleep(0.5)
            
            # Verify sensor
            quat = bno.quaternion
            if quat is None:
                raise RuntimeError("Unable to read quaternion data")
            
            # Test quaternion conversion
            roll, pitch, yaw = quaternion_to_euler(*quat)
            if any(x is None for x in [roll, pitch, yaw]):
                raise RuntimeError("Invalid quaternion conversion")
                
            print("IMU initialized successfully")
            return bno
                
        except Exception as e:
            print(f"Attempt {attempt + 1} failed: {str(e)}")
            if attempt < max_retries - 1:
                print(f"Retrying in {retry_delay} seconds...")
                time.sleep(retry_delay)
            else:
                print("IMU initialization failed after all attempts")
                raise

def calibrate_imu(bno):
    """Calibrate IMU with improved error handling"""
    print("\nCalibrating IMU (stay still)...")
    samples = []
    max_samples = 50
    timeout = 30  # seconds
    start_time = time.time()
    
    while len(samples) < max_samples and time.time() - start_time < timeout:
        try:
            if len(samples) % 10 == 0 and len(samples) != 0:
                print(f"Collecting samples... {len(samples)}/{max_samples}", end='\r')
            quat = bno.quaternion
            
            if quat is not None and len(quat) == 4:
                _, _, yaw = quaternion_to_euler(*quat)
                if yaw is not None and not math.isnan(yaw):
                    samples.append(yaw)
            else:
                print("\nInvalid quaternion reading, retrying...")
                time.sleep(0.5)
                
        except Exception as e:
            print(f"\nError during calibration: {str(e)}")
            time.sleep(0.5)
    
    if len(samples) >= max_samples / 2:
        calibration_offset = sum(samples) / len(samples)
        print(f"\nCalibration complete. Reference yaw: {calibration_offset:.2f}°")
        return calibration_offset
    else:
        raise RuntimeError(f"Failed to collect enough valid samples (got {len(samples)} of {max_samples})")

def get_current_yaw(bno, calibration_offset, retries=5, delay=0.1):
    """Get current yaw with retry mechanism"""
    for attempt in range(retries):
        try:
            quat = bno.quaternion
            if quat is not None and len(quat) == 4 and not any(math.isnan(x) for x in quat):
                _, _, yaw = quaternion_to_euler(*quat)
                if yaw is not None:
                    yaw_adjusted = (yaw - calibration_offset + 360) % 360
                    if yaw_adjusted > 180:
                        yaw_adjusted -= 360  # Convert to range [-180, 180]
                    return yaw_adjusted
            print(f"IMU read attempt {attempt + 1} failed, retrying...")
            time.sleep(delay)
        except Exception as e:
            print(f"Exception during IMU read: {e}")
            time.sleep(delay)
    print("Failed to get current yaw after multiple attempts.")
    return None

class GaitGenerator:
    def __init__(self, motors, mcp):
        self.motors = motors
        self.mcp = mcp
        self.start_time = time.time()

    def generate_sawtooth_position(self, period=SAWTOOTH_PERIOD, max_angle=MAX_ANGLE):
        """Generate a sawtooth wave position for gait."""
        elapsed_time = time.time() - self.start_time
        position_in_cycle = (elapsed_time % period) / period
        position = (position_in_cycle * max_angle) % max_angle
        return position

    def update_gait(self):
        """Update motor positions based on gait."""
        base_position = self.generate_sawtooth_position()
        # Synchronize Motor Groups with 180-degree phase difference
        group1 = ['M2', 'M3']  # Right Front and Left Rear
        group2 = ['M1', 'M4']  # Left Front and Right Rear

        # Calculate target positions
        target_group1 = base_position % 360
        target_group2 = (base_position + 180) % 360

        # Move motors in group1
        for motor in group1:
            self.motors[motor].move_to_position(target_group1, self.mcp)

        # Move motors in group2
        for motor in group2:
            self.motors[motor].move_to_position(target_group2, self.mcp)

def perform_point_turn(motors, turn_direction, angle, bno, calibration_offset):
    """
    Perform a point turn to correct the robot's direction.

    :param motors: Dictionary of MotorController instances.
    :param turn_direction: 'left' or 'right'.
    :param angle: Angle to turn in degrees.
    :param bno: IMU sensor object.
    :param calibration_offset: Initial yaw offset.
    """
    def angular_difference(target, current):
        """Compute the minimal angular difference from current to target in degrees."""
        diff = (target - current + 180) % 360 - 180
        return diff

    motor_speed = 70  # Speed for point turn; adjust as needed

    # Configure motors for point turn
    if turn_direction == 'right':
        # Right point turn: Right motors backward, Left motors forward
        motors['M1'].set_motor_direction('forward')
        motors['M2'].set_motor_direction('backward')
        motors['M3'].set_motor_direction('forward')
        motors['M4'].set_motor_direction('backward')
    else:
        # Left point turn: Left motors backward, Right motors forward
        motors['M1'].set_motor_direction('backward')
        motors['M2'].set_motor_direction('forward')
        motors['M3'].set_motor_direction('backward')
        motors['M4'].set_motor_direction('forward')

    # Set motor speeds
    for motor in motors.values():
        motor.set_motor_speed(motor_speed)

    print(f"\nStarting {turn_direction} point turn of {angle}°")

    # Record initial yaw
    initial_yaw = None
    attempts = 0
    while initial_yaw is None and attempts < 5:
        initial_yaw = get_current_yaw(bno, calibration_offset)
        if initial_yaw is None:
            print("Failed to get initial yaw for point turn, retrying...")
            time.sleep(0.1)
        attempts += 1
    if initial_yaw is None:
        print("Failed to get initial yaw after multiple attempts, aborting turn.")
        # Stop motors
        for motor in motors.values():
            motor.stop_motor()
        return

    # Determine target yaw
    if turn_direction == 'right':
        target_yaw = (initial_yaw + angle) % 360
    else:
        target_yaw = (initial_yaw - angle + 360) % 360

    # Main turn loop
    while True:
        current_yaw = get_current_yaw(bno, calibration_offset)
        if current_yaw is None:
            print("Yaw reading failed during point turn.")
            # Optionally retry or stop motors
            break

        angle_remaining = angular_difference(target_yaw, current_yaw)

        print(f"Point Turn - Current yaw: {current_yaw:.2f}°, Angle remaining: {angle_remaining:.2f}°    ", end='\r')

        if abs(angle_remaining) <= 2.0:
            print(f"\nPoint turn completed! Final yaw: {current_yaw:.2f}°")
            break

        time.sleep(0.05)

    # Stop motors after point turn
    for motor in motors.values():
        motor.stop_motor()
    time.sleep(0.5)  # Brief pause after turn

def stop_all_motors(motor_pins, motor_pwms):
    """Stop all motors."""
    for i, _ in enumerate(motor_pins, 1):
        motor = f"M{i}"
        # Stop motor direction
        in1, in2, _ = motor_pins[i-1]
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        # Stop motor speed
        motor_pwms[motor].ChangeDutyCycle(0)

def main():
    parser = argparse.ArgumentParser(description='Quadruped Robot Controller with AprilTag Detection and IMU Correction')
    parser.add_argument('--manual_turn', type=float, default=0.0,
                        help='Manual turn angle in degrees (positive for right, negative for left)')
    args = parser.parse_args()

    # Initialize motors, IMU, and other components
    motor_pins, motor_pwms = setup_motors()
    motors = {
        'M1': MotorController("M1", MOTOR1_IN1, MOTOR1_IN2, motor_pwms['M1'], MOTOR1_ADC_CHANNEL, encoder_flipped=False),
        'M2': MotorController("M2", MOTOR2_IN1, MOTOR2_IN2, motor_pwms['M2'], MOTOR2_ADC_CHANNEL, encoder_flipped=True),
        'M3': MotorController("M3", MOTOR3_IN1, MOTOR3_IN2, motor_pwms['M3'], MOTOR3_ADC_CHANNEL, encoder_flipped=False),
        'M4': MotorController("M4", MOTOR4_IN1, MOTOR4_IN2, motor_pwms['M4'], MOTOR4_ADC_CHANNEL, encoder_flipped=True)
    }

    bno = setup_imu()
    calibration_offset = calibrate_imu(bno)
    mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))
    gait = GaitGenerator(motors, mcp)

    # Initialize camera with optimal settings
    picam2 = Picamera2()
    camera_config = picam2.create_still_configuration(
        main={"size": (1920, 1080)},
        controls={
            "FrameDurationLimits": (16666, 16666),  # ~60fps
            "ExposureTime": 8000,
            "AnalogueGain": 2.5,
            "Brightness": 0.5,
            "Contrast": 1.2,
            "Sharpness": 2.0
        }
    )
    picam2.configure(camera_config)
    picam2.start()

    # Initialize AprilTag detector
    at_detector = Detector(
        families='tag36h11',
        nthreads=os.cpu_count() or 1,
        quad_decimate=2.0,
        quad_sigma=0.8,
        refine_edges=True,
        decode_sharpening=0.5,
        debug=0
    )

    # Control parameters
    YAW_THRESHOLD = 15.0
    CORRECTION_ANGLE = 15.0
    distance_threshold = 0.5
    stop_event = threading.Event()

    # Graceful shutdown handler
    def cleanup(signum, frame):
        print("\nShutting down gracefully...")
        stop_event.set()
        time.sleep(0.5)  # Give time for threads to complete and the zip file to close
        try:
            picam2.stop()
            stop_all_motors(motor_pins, motor_pwms)
            for pwm in motor_pwms.values():
                pwm.stop()
            GPIO.cleanup()
        except Exception as e:
            print(f"Error during cleanup: {e}")
            traceback.print_exc()
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)

    # Main loop to perform the operation four times
    for iteration in range(4):
        print(f"\nStarting iteration {iteration+1}/4")

        # Reset stop_event and distance_to_tag
        stop_event.clear()
        distance_to_tag = None

        # Create timestamp for unique zip filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        zip_filename = f"captured_frames_iter{iteration+1}_{timestamp}.zip"

        # Start image processing thread with integrated zipping
        def image_processing_thread():
            nonlocal distance_to_tag
            print("Starting image processing thread...")
            
            last_valid_distance = None
            consecutive_detections = 0
            frame_count = 0
            start_time = time.time()
            last_save_time = time.time()
            save_interval = 0.5  # Save one frame every 0.5 seconds when no tags detected
            
            with zipfile.ZipFile(zip_filename, 'w', zipfile.ZIP_DEFLATED) as zipf:
                while not stop_event.is_set():
                    try:
                        frame = picam2.capture_array()
                        
                        # Calculate FPS
                        frame_count += 1
                        if frame_count % 60 == 0:
                            elapsed = time.time() - start_time
                            fps = frame_count / elapsed
                            print(f"Actual FPS: {fps:.2f}")
                        
                        # Convert to grayscale for AprilTag detection
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        
                        # Detect AprilTags
                        tags = at_detector.detect(
                            gray,
                            estimate_tag_pose=True,
                            camera_params=CAMERA_PARAMS,
                            tag_size=TAG_SIZE
                        )
                        
                        # Determine if we should save this frame
                        should_save = False
                        if tags:
                            # Always save frames with detected tags
                            should_save = True
                        elif time.time() - last_save_time >= save_interval:
                            # Save periodic frames when no tags detected
                            should_save = True
                            last_save_time = time.time()
                        
                        # Save frame if needed
                        if should_save:
                            success, buffer = cv2.imencode('.jpg', frame)
                            if success:
                                frame_timestamp = datetime.now().strftime("%H%M%S_%f")
                                if tags:
                                    best_tag = max(tags, key=lambda x: x.decision_margin)
                                    frame_filename = f"frame_{frame_timestamp}_tag{best_tag.tag_id}_dist{float(best_tag.pose_t[2]):.2f}m.jpg"
                                else:
                                    frame_filename = f"frame_{frame_timestamp}_notag.jpg"
                                
                                zipf.writestr(frame_filename, buffer.tobytes())
                        
                        # Process AprilTag detections
                        if tags:
                            best_tag = max(tags, key=lambda x: x.decision_margin)
                            translation = best_tag.pose_t
                            current_distance = float(translation[2])

                            # Print the detected distance
                            print(f"Tag {best_tag.tag_id} detected at {current_distance:.2f}m")

                            if last_valid_distance is not None:
                                current_distance = 0.7 * current_distance + 0.3 * last_valid_distance
                            
                            last_valid_distance = current_distance
                            consecutive_detections += 1
                            
                            if consecutive_detections >= 2:
                                distance_to_tag = current_distance
                                
                                if current_distance <= distance_threshold:
                                    # Signal that the tag is reached
                                    print(f"Target reached. Distance: {current_distance:.2f}m")
                                    stop_event.set()  # This will signal the main thread to proceed with the turn
                                    break
                        else:
                            consecutive_detections = 0
                        
                        time.sleep(0.016)  # ~60 FPS
                        
                    except Exception as e:
                        print(f"Error in image processing: {e}")
                        traceback.print_exc()
                        time.sleep(0.1)
            
            print(f"Image processing thread stopped. Images saved to {zip_filename}")

        # Start the image processing thread
        threading.Thread(target=image_processing_thread, daemon=True).start()

        print("\nStarting combined operation with image capture...")

        try:
            # Initial manual turn if specified (only for the first iteration)
            if iteration == 0 and args.manual_turn != 0.0:
                turn_direction = 'right' if args.manual_turn > 0 else 'left'
                target_angle = abs(args.manual_turn)
                perform_point_turn(motors, turn_direction, target_angle, bno, calibration_offset)

            last_correction_time = time.time()
            correction_cooldown = 1.0

            while not stop_event.is_set():
                # Update gait
                gait.update_gait()

                # Check IMU for straight-line correction
                current_time = time.time()
                if current_time - last_correction_time >= correction_cooldown:
                    current_yaw = get_current_yaw(bno, calibration_offset)
                    
                    if current_yaw is not None:
                        if current_yaw > YAW_THRESHOLD:
                            print(f"\nCorrecting right drift: {current_yaw:.2f}°")
                            perform_point_turn(motors, 'left', CORRECTION_ANGLE, bno, calibration_offset)
                            last_correction_time = current_time
                        elif current_yaw < -YAW_THRESHOLD:
                            print(f"\nCorrecting left drift: {current_yaw:.2f}°")
                            perform_point_turn(motors, 'right', CORRECTION_ANGLE, bno, calibration_offset)
                            last_correction_time = current_time

                time.sleep(0.02)

            # Ensure main loop exits when stop_event is set
            print("\nMain loop exited.")

            # Handle stop condition
            print("\nTarget distance reached. Stopping and turning...")
            stop_all_motors(motor_pins, motor_pwms)
            time.sleep(1)  # Allow all motors to stop completely

            # Perform the 90-degree turn
            try:
                perform_point_turn(motors, 'right', 90.0, bno, calibration_offset)
                print("90-degree turn completed.")
            except Exception as e:
                print(f"Error during final turn: {e}")
                traceback.print_exc()

            # Recalibrate the IMU after the turn
            try:
                calibration_offset = calibrate_imu(bno)
                print("IMU recalibrated.")
            except Exception as e:
                print(f"Error during IMU recalibration: {e}")
                traceback.print_exc()

        except Exception as e:
            print(f"\nUnexpected error: {e}")
            traceback.print_exc()
            break  # Exit the loop if an unexpected error occurs

    # After the loop is done, proceed to cleanup
    cleanup(None, None)

if __name__ == "__main__":
    main()
