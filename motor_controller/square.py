import warnings
import argparse
import sys
import logging
import math
import time
import signal
import threading
import os

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

# Suppress specific warnings
warnings.filterwarnings("ignore", category=RuntimeWarning)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(message)s',
    handlers=[logging.StreamHandler()]
)

# GPIO setup and constants
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Camera and AprilTag constants
FX = 1435.0
FY = 1440.0
CX = 1296.0
CY = 972.0
CAMERA_PARAMS = (FX, FY, CX, CY)
TAG_SIZE = 0.092  # Tag size in meters
DISTANCE_THRESHOLD = 0.5  # Target distance in meters

# Synchronization events
stop_event = threading.Event()
turn_completed_event = threading.Event()

# Motor Pins (Adjust according to your setup)
MOTOR_PINS = {
    "M1": {"in1": 4, "in2": 7, "spd": 24, "adc_channel": 0},
    "M2": {"in1": 5, "in2": 25, "spd": 6, "adc_channel": 1},
    "M3": {"in1": 17, "in2": 12, "spd": 13, "adc_channel": 2},
    "M4": {"in1": 18, "in2": 27, "spd": 19, "adc_channel": 3}
}

# Output ZIP file for captured frames
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
zip_filename = f"captured_frames_{timestamp}.zip"

def perform_point_turn(motors, turn_direction, angle, imu, calibration_offset):
    """Perform a point turn based on IMU."""
    print(f"Starting {turn_direction} point turn of {angle}°")
    motor_speed = 70  # Adjust motor speed for turn

    # Configure motors for turning
    if turn_direction == 'right':
        motors['M1'].set_motor_direction('forward')
        motors['M2'].set_motor_direction('backward')
        motors['M3'].set_motor_direction('forward')
        motors['M4'].set_motor_direction('backward')
    else:
        motors['M1'].set_motor_direction('backward')
        motors['M2'].set_motor_direction('forward')
        motors['M3'].set_motor_direction('backward')
        motors['M4'].set_motor_direction('forward')

    # Start motors
    for motor in motors.values():
        motor.set_motor_speed(motor_speed)

    # Monitor yaw angle for completion
    initial_yaw = get_current_yaw(imu, calibration_offset)
    if initial_yaw is None:
        print("Error: Could not read IMU yaw for turn.")
        return

    target_yaw = (initial_yaw + angle) % 360 if turn_direction == 'right' else (initial_yaw - angle + 360) % 360
    if target_yaw > 180:
        target_yaw -= 360  # Normalize to [-180, 180]

    while True:
        current_yaw = get_current_yaw(imu, calibration_offset)
        if current_yaw is None:
            print("Error: Could not read IMU yaw during turn.")
            break

        if abs(current_yaw - target_yaw) <= 2:  # Tolerance of 2 degrees
            print(f"Point turn completed. Final yaw: {current_yaw:.2f}°")
            break

        time.sleep(0.05)

    # Stop all motors after the turn
    for motor in motors.values():
        motor.stop_motor()
    time.sleep(0.5)  # Short delay to stabilize after the turn

    turn_completed_event.set()  # Signal that the turn is completed


def image_processing_thread(picam2, at_detector, motors, imu, calibration_offset):
    """Thread for processing AprilTags and triggering behavior."""
    global stop_event

    with zipfile.ZipFile(zip_filename, 'w', zipfile.ZIP_DEFLATED) as zipf:
        while not stop_event.is_set():
            try:
                frame = picam2.capture_array()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # Detect AprilTags
                tags = at_detector.detect(
                    gray,
                    estimate_tag_pose=True,
                    camera_params=CAMERA_PARAMS,
                    tag_size=TAG_SIZE
                )

                if tags:
                    best_tag = max(tags, key=lambda x: x.decision_margin)
                    distance_to_tag = float(best_tag.pose_t[2])  # Z-distance

                    if distance_to_tag <= DISTANCE_THRESHOLD:
                        print(f"Target reached at {distance_to_tag:.2f}m")
                        stop_event.set()  # Signal the main loop to stop
                        break

                time.sleep(0.05)

            except Exception as e:
                print(f"Error in image processing thread: {e}")
                time.sleep(0.1)

def main():
    """Main control logic."""
    global stop_event, turn_completed_event

    # Initialize components
    picam2 = Picamera2()
    picam2.configure(picam2.create_still_configuration(main={"size": (1920, 1080)}))
    picam2.start()

    at_detector = Detector(families='tag36h11', quad_decimate=2.0)

    # Start image processing thread
    threading.Thread(
        target=image_processing_thread,
        args=(picam2, at_detector, None, None, None),
        daemon=True
    ).start()

    # Wait for stop event or other termination condition
    while not stop_event.is_set():
        time.sleep(0.1)

    # Perform the 90-degree turn after detecting the tag
    print("Performing 90-degree turn...")
    perform_point_turn(None, 'right', 90, None, None)

    # Wait for turn to complete
    turn_completed_event.wait()

    # Stop and cleanup
    picam2.stop()
    print("Operation completed. Shutting down.")

if __name__ == "__main__":
    main()
