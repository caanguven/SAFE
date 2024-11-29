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

class ImageProcessor:
    def __init__(self, camera, detector, distance_threshold=0.5):
        self.camera = camera
        self.detector = detector
        self.distance_threshold = distance_threshold
        self.tag_detected = False
        self.target_reached = False
        
    def process_frame(self):
        frame = self.camera.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=TAG_SIZE
        )
        
        if tags:
            best_tag = max(tags, key=lambda x: x.decision_margin)
            distance = float(best_tag.pose_t[2])
            print(f"Tag {best_tag.tag_id} detected at {distance:.2f}m")
            self.tag_detected = True
            
            if distance <= self.distance_threshold:
                self.target_reached = True
                return True
        
        return False

def main():
    # Initialize all your hardware and parameters as before
    # [Previous initialization code remains the same until the main loop]

    # Initialize image processor
    processor = ImageProcessor(picam2, at_detector)
    
    # State tracking
    turn_count = 0
    MAX_TURNS = 4

    try:
        while turn_count < MAX_TURNS:
            # Forward movement phase
            print(f"\nStarting forward movement {turn_count + 1}/4")
            target_reached = False
            
            while not target_reached:
                # Update walking gait
                gait.update_gait()
                
                # Process camera feed
                target_reached = processor.process_frame()
                
                # Sleep briefly
                time.sleep(0.02)
            
            # Target reached, stop and turn
            print(f"\nTarget reached for leg {turn_count + 1}/4")
            stop_all_motors(motor_pins, motor_pwms)
            time.sleep(1)  # Ensure complete stop
            
            # Perform 90-degree turn
            print(f"\nExecuting turn {turn_count + 1}/4")
            perform_point_turn(motors, 'right', 90.0, bno, calibration_offset)
            turn_count += 1
            
            if turn_count < MAX_TURNS:
                print(f"\nCompleted turn {turn_count}/4. Continuing to next leg.")
                time.sleep(1)  # Brief pause before next leg
            else:
                print("\nAll turns completed! Mission accomplished.")
        
        print("\nSquare pattern completed successfully!")
        
    except Exception as e:
        print(f"\nError during execution: {e}")
        traceback.print_exc()
    finally:
        # Cleanup
        print("\nCleaning up...")
        picam2.stop()
        stop_all_motors(motor_pins, motor_pwms)
        for pwm in motor_pwms.values():
            pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()