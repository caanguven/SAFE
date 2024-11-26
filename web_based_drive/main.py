# # app.py

# from flask import Flask, Response, request, render_template, jsonify
# from motor_control import MotorControlSystem  # Assuming motor_control.py is in the same directory
# from picamera2 import Picamera2
# import cv2
# import numpy as np
# from pupil_apriltags import Detector
# import subprocess
# import threading
# import time
# import math
# import os
# import logging

# app = Flask(__name__)

# # Configure Logging
# logging.basicConfig(level=logging.DEBUG,
#                     format='%(asctime)s [%(levelname)s] %(message)s',
#                     handlers=[
#                         logging.FileHandler("app.log"),
#                         logging.StreamHandler()
#                     ])

# # Initialize Motor Control System
# motor_system = MotorControlSystem(mode='normal')

# # Initialize Camera
# class Camera:
#     instance = None

#     @staticmethod
#     def get_instance():
#         if Camera.instance is None:
#             logging.info("Initializing camera...")
#             Camera.instance = Picamera2()
#             Camera.instance.configure(Camera.instance.create_preview_configuration(main={"size": (640, 480)}))
#             Camera.instance.start()
#         else:
#             logging.info("Camera already initialized.")
#         return Camera.instance

#     @staticmethod
#     def release_instance():
#         if Camera.instance is not None:
#             logging.info("Releasing camera...")
#             Camera.instance.stop()
#             Camera.instance.close()
#             Camera.instance = None

# # Initialize AprilTag Detector
# at_detector = Detector(families='tag36h11 tag52h13',
#                        nthreads=1,
#                        quad_decimate=1.0,
#                        quad_sigma=0.0,
#                        refine_edges=1,
#                        decode_sharpening=0.25,
#                        debug=0)

# # Specify the path to the Haar cascade XML file for face detection
# haar_cascade_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'

# # Load the Haar cascade for face detection
# face_cascade = cv2.CascadeClassifier(haar_cascade_path)

# # Check if the cascade loaded successfully
# if face_cascade.empty():
#     logging.error('Failed to load Haar cascade classifier for face detection.')
# else:
#     logging.info('Haar cascade classifier for face detection loaded successfully.')

# # Video Streaming Generator with Face and AprilTag Detection
# def gen(camera):
#     while True:
#         try:
#             frame = camera.capture_array()

#             # Convert to grayscale for face detection
#             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#             faces = face_cascade.detectMultiScale(gray, 1.1, 4)

#             # Detect AprilTags
#             tags = at_detector.detect(gray, estimate_tag_pose=False, camera_params=None, tag_size=None)

#             # Draw rectangles around the detected faces
#             for (x, y, w, h) in faces:
#                 cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
#                 cv2.putText(frame, "Face", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

#             # Process AprilTags
#             for tag in tags:
#                 # Corner points
#                 bottom_left = (int(tag.corners[0][0]), int(tag.corners[0][1]))
#                 top_right = (int(tag.corners[2][0]), int(tag.corners[2][1]))

#                 # Draw the rectangle
#                 cv2.rectangle(frame, bottom_left, top_right, (0, 255, 0), 2)

#                 # Draw the tag ID
#                 center = (int(tag.center[0]), int(tag.center[1]))
#                 cv2.putText(frame, f"ID {tag.tag_id}", center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

#             # Encode the frame as JPEG
#             _, jpeg = cv2.imencode('.jpg', frame)
#             frame_bytes = jpeg.tobytes()

#             # Yield frame in byte format
#             yield (b'--frame\r\n'
#                    b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
#         except Exception as e:
#             logging.error(f"Error in video streaming: {e}")
#             break

# @app.route('/')
# def index():
#     return render_template('index.html')

# @app.route('/video_feed')
# def video_feed():
#     picam2 = Camera.get_instance()
#     return Response(gen(picam2),
#                     mimetype='multipart/x-mixed-replace; boundary=frame')

# @app.route('/control', methods=['POST'])
# def control():
#     try:
#         data = request.json
#         direction = data.get('direction', 'stable')
#         if direction not in ['forward', 'backward', 'left', 'right', 'stable']:
#             raise ValueError("Invalid direction received.")
#         motor_system.set_direction(direction)
#         logging.info(f"Control command received: direction='{direction}'")
#         return jsonify({'status': 'success', 'direction': direction})
#     except Exception as e:
#         logging.error(f"Error in /control: {e}")
#         return jsonify({'status': 'error', 'message': str(e)}), 400

# @app.route('/mode', methods=['POST'])
# def mode():
#     try:
#         data = request.json
#         mode = data.get('mode', 'normal')
#         if mode not in ['normal', 'gallop']:
#             raise ValueError("Invalid mode received.")
#         motor_system.set_mode(mode)
#         logging.info(f"Mode command received: mode='{mode}'")
#         return jsonify({'status': 'success', 'mode': mode})
#     except Exception as e:
#         logging.error(f"Error in /mode: {e}")
#         return jsonify({'status': 'error', 'message': str(e)}), 400

# @app.route('/status', methods=['GET'])
# def status():
#     try:
#         status = motor_system.get_status()
#         logging.debug("Status requested.")
#         return jsonify(status)
#     except Exception as e:
#         logging.error(f"Error in /status: {e}")
#         return jsonify({'status': 'error', 'message': str(e)}), 400

# @app.route('/shutdown', methods=['POST'])
# def shutdown():
#     """Route to gracefully shutdown the Flask application."""
#     try:
#         shutdown_func = request.environ.get('werkzeug.server.shutdown')
#         if shutdown_func is None:
#             raise RuntimeError('Not running with the Werkzeug Server')
#         shutdown_func()
#         motor_system.stop()
#         Camera.release_instance()
#         logging.info("Shutting down Flask application.")
#         return 'Server shutting down...', 200
#     except Exception as e:
#         logging.error(f"Error during shutdown: {e}")
#         return 'Shutdown failed.', 500

# if __name__ == '__main__':
#     try:
#         app.run(host='0.0.0.0', port=5000, threaded=True)
#     except KeyboardInterrupt:
#         logging.info("KeyboardInterrupt received. Shutting down.")
#         motor_system.stop()
#         Camera.release_instance()



# app.py

from flask import Flask, Response, request, render_template, jsonify
from motor_control import MotorControlSystem  # Assuming motor_control.py is in the same directory
from picamera2 import Picamera2
import cv2
import numpy as np
from pupil_apriltags import Detector
import subprocess
import threading
import time
import math
import os
import logging
import busio
import board
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from threading import Lock
import RPi.GPIO as GPIO


app = Flask(__name__)

# Configure Logging
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s [%(levelname)s] %(message)s',
                    handlers=[
                        logging.FileHandler("app.log"),
                        logging.StreamHandler()
                    ])

try:
    GPIO.cleanup()
except:
    pass
        
        # GPIO setup
GPIO.setwarnings(False)  # Disable warnings
GPIO.setmode(GPIO.BOARD)

# Initialize Motor Control System
motor_system = MotorControlSystem(mode='normal')

imu_data = {'roll': 0, 'pitch': 0, 'yaw': 0}
calibration_offsets = {'roll': None, 'pitch': None, 'yaw': None}
calibrated = False
imu_data_lock = Lock()


# Gyro 
try:
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    bno = BNO08X_I2C(i2c)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    logging.info("IMU initialized successfully")
except Exception as e:
    logging.error(f"Error initializing IMU: {e}")

# Add these functions
def quaternion_to_euler(quat_i, quat_j, quat_k, quat_real):
    roll = math.atan2(2 * (quat_real * quat_i + quat_j * quat_k),
                      1 - 2 * (quat_i**2 + quat_j**2))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (quat_real * quat_j - quat_k * quat_i))))
    yaw = math.atan2(2 * (quat_real * quat_k + quat_i * quat_j),
                     1 - 2 * (quat_j**2 + quat_k**2))
    return roll, pitch, yaw

def update_imu_data():
    global calibrated
    while True:
        try:
            if not calibrated:
                quat_i, quat_j, quat_k, quat_real = bno.quaternion
                roll, pitch, yaw = quaternion_to_euler(quat_i, quat_j, quat_k, quat_real)
                calibration_offsets['roll'] = math.degrees(roll)
                calibration_offsets['pitch'] = math.degrees(pitch)
                calibration_offsets['yaw'] = math.degrees(yaw)
                calibrated = True
                
            quat_i, quat_j, quat_k, quat_real = bno.quaternion
            roll, pitch, yaw = quaternion_to_euler(quat_i, quat_j, quat_k, quat_real)
            
            with imu_data_lock:
                imu_data['roll'] = math.degrees(roll) - calibration_offsets['roll']
                imu_data['pitch'] = math.degrees(pitch) - calibration_offsets['pitch']
                imu_data['yaw'] = math.degrees(yaw) - calibration_offsets['yaw']
            
        except Exception as e:
            logging.error(f"Error updating IMU data: {e}")
        time.sleep(0.1)

# Add this route
@app.route('/imu_data')
def get_imu_data():
    with imu_data_lock:
        return jsonify(imu_data)

# Modify the if __name__ == '__main__': section
if __name__ == '__main__':
    try:
        # Start IMU update thread
        imu_thread = threading.Thread(target=update_imu_data, daemon=True)
        imu_thread.start()
        app.run(host='0.0.0.0', port=5000, threaded=True)
    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt received. Shutting down.")
        motor_system.stop()
        Camera.release_instance()

# Initialize Camera
class Camera:
    instance = None

    @staticmethod
    def get_instance():
        if Camera.instance is None:
            logging.info("Initializing camera...")
            Camera.instance = Picamera2()
            Camera.instance.configure(Camera.instance.create_preview_configuration(main={"size": (640, 480)}))
            Camera.instance.start()
        else:
            logging.info("Camera already initialized.")
        return Camera.instance

    @staticmethod
    def release_instance():
        if Camera.instance is not None:
            logging.info("Releasing camera...")
            Camera.instance.stop()
            Camera.instance.close()
            Camera.instance = None

# Initialize AprilTag Detector
at_detector = Detector(families='tag36h11 tag52h13',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

# Specify the path to the Haar cascade XML file for face detection
haar_cascade_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'

# Load the Haar cascade for face detection
face_cascade = cv2.CascadeClassifier(haar_cascade_path)

# Check if the cascade loaded successfully
if face_cascade.empty():
    logging.error('Failed to load Haar cascade classifier for face detection.')
else:
    logging.info('Haar cascade classifier for face detection loaded successfully.')

# Video Streaming Generator with Face and AprilTag Detection
def gen(camera):
    while True:
        try:
            frame = camera.capture_array()

            # Convert to grayscale for face detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, 1.1, 4)

            # Detect AprilTags
            tags = at_detector.detect(gray, estimate_tag_pose=False, camera_params=None, tag_size=None)

            # Draw rectangles around the detected faces
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(frame, "Face", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

            # Process AprilTags
            for tag in tags:
                # Corner points
                bottom_left = (int(tag.corners[0][0]), int(tag.corners[0][1]))
                top_right = (int(tag.corners[2][0]), int(tag.corners[2][1]))

                # Draw the rectangle
                cv2.rectangle(frame, bottom_left, top_right, (0, 255, 0), 2)

                # Draw the tag ID
                center = (int(tag.center[0]), int(tag.center[1]))
                cv2.putText(frame, f"ID {tag.tag_id}", center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Encode the frame as JPEG
            _, jpeg = cv2.imencode('.jpg', frame)
            frame_bytes = jpeg.tobytes()

            # Yield frame in byte format
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        except Exception as e:
            logging.error(f"Error in video streaming: {e}")
            break

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    picam2 = Camera.get_instance()
    return Response(gen(picam2),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/control', methods=['POST'])
def control():
    try:
        data = request.json
        direction = data.get('direction', 'stable')
        if direction not in ['forward', 'backward', 'left', 'right', 'stable']:
            raise ValueError("Invalid direction received.")
        motor_system.set_direction(direction)
        logging.info(f"Control command received: direction='{direction}'")
        return jsonify({'status': 'success', 'direction': direction})
    except Exception as e:
        logging.error(f"Error in /control: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 400

@app.route('/mode', methods=['POST'])
def mode():
    try:
        data = request.json
        mode = data.get('mode', 'normal')
        if mode not in ['normal', 'gallop']:
            raise ValueError("Invalid mode received.")
        motor_system.set_mode(mode)
        logging.info(f"Mode command received: mode='{mode}'")
        return jsonify({'status': 'success', 'mode': mode})
    except Exception as e:
        logging.error(f"Error in /mode: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 400

@app.route('/status', methods=['GET'])
def status():
    try:
        status = motor_system.get_status()
        logging.debug("Status requested.")
        return jsonify(status)
    except Exception as e:
        logging.error(f"Error in /status: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 400

@app.route('/shutdown', methods=['POST'])
def shutdown():
    """Route to gracefully shutdown the Flask application."""
    try:
        shutdown_func = request.environ.get('werkzeug.server.shutdown')
        if shutdown_func is None:
            raise RuntimeError('Not running with the Werkzeug Server')
        shutdown_func()
        motor_system.stop()
        Camera.release_instance()
        logging.info("Shutting down Flask application.")
        return 'Server shutting down...', 200
    except Exception as e:
        logging.error(f"Error during shutdown: {e}")
        return 'Shutdown failed.', 500

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, threaded=True)
    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt received. Shutting down.")
        motor_system.stop()
        Camera.release_instance()
