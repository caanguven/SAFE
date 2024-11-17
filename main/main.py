from flask import Flask, Response, request, render_template, jsonify, stream_with_context
from picamera2 import Picamera2
import cv2
import numpy as np
from pupil_apriltags import Detector
import subprocess
import threading
import time
import math
import os
import busio
import board
from threading import Lock
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR




app = Flask(__name__)

# Specify the path to the Haar cascade XML file
haar_cascade_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'

# Load the Haar cascade for face detection
face_cascade = cv2.CascadeClassifier(haar_cascade_path)
motor_process = None  # Initialize motor_process as None

# Check if the cascade loaded successfully
if face_cascade.empty():
    print('Failed to load cascade classifier')
else:
    print('Cascade classifier loaded successfully')

# Initialize BNO08x IMU
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
bno = BNO08X_I2C(i2c)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

# Global variable for IMU data
imu_data = {'roll': 0, 'pitch': 0, 'yaw': 0}
calibration_offsets = {'roll': None, 'pitch': None, 'yaw': None}
calibrated = False

imu_data_lock = threading.Lock()


# Initialize the AprilTag libraries
at_detector = Detector(families='tag36h11 tag52h13',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

screen_center = (320, 240)
screen_center_x = 320  # Assuming a 640x480 resolution

# Singleton pattern for Picamera2 instance
class Camera:
    instance = None

    @staticmethod
    def get_instance():
        if Camera.instance is None:
            print("Initializing camera...")
            Camera.instance = Picamera2()
            Camera.instance.configure(Camera.instance.create_preview_configuration(main={"size": (640, 480)}))
            Camera.instance.start()
        else:
            print("Camera already initialized.")
        return Camera.instance

    @staticmethod
    def release_instance():
        if Camera.instance is not None:
            print("Releasing camera...")
            Camera.instance.stop()
            Camera.instance.close()
            Camera.instance = None

def gen(camera):
    """Video streaming generator function with AprilTag detection."""
    while True:
        frame = camera.capture_array()

        # Converts to gray
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = at_detector.detect(gray, estimate_tag_pose=False, camera_params=None, tag_size=None)

        # Process
        for tag in tags:
            # Corner points
            bottom_left = (int(tag.corners[0][0]), int(tag.corners[0][1]))
            top_right = (int(tag.corners[2][0]), int(tag.corners[2][1]))

            # Draw the rectangle
            cv2.rectangle(frame, bottom_left, top_right, (0, 255, 0), 2)

            # Draw the tag ID
            center = (int(tag.center[0]), int(tag.center[1]))
            cv2.putText(frame, f"ID {tag.tag_id}", center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Pose of the tag
            relative_position = (center[0] - screen_center[0], -(center[1] - screen_center[1]))

            # Show relative pose
            cv2.putText(frame, f"Rel Pos: {relative_position}", (center[0] + 10, center[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Get the image per frame
        _, jpeg_frame = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg_frame.tobytes() + b'\r\n')

def detect_april_tag_direction(camera):
    """Detect AprilTag and determine if it's on the left or right side of the screen."""
    while True:
        frame = camera.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = at_detector.detect(gray, estimate_tag_pose=False, camera_params=None, tag_size=None)

        direction = "None"
        magnitude = 0

        for tag in tags:
            tag_x_position = int(tag.center[0])

            # Calculate the magnitude from the center
            distance_from_center = tag_x_position - screen_center_x
            magnitude = (abs(distance_from_center) / screen_center_x) * 10

            if distance_from_center < 0:
                direction = f"Left: Magnitude {magnitude:.2f}"
            else:
                direction = f"Right: Magnitude {magnitude:.2f}"

        yield f"data: {direction}\n\n"

@app.route('/')
def main_page():
    return render_template('main.html')

@app.route('/control_motor', methods=['POST'])
def control_motor():
    global motor_process

    try:
        data = request.get_json()
        print(f"Received data from frontend: {data}")  # Log the received data

        direction = data.get('direction')
        print(f"Received direction: {direction}")  # Log the direction

        # If there's an existing motor process, terminate it
        if motor_process and motor_process.poll() is None:
            print("Stopping previous motor process")
            motor_process.terminate()
            motor_process = None

        # Depending on the direction, start a new motor control process
        if direction == 'forward':
            print("Moving forward")
            motor_process = subprocess.Popen(['sudo', '-E', 'python', 'motor_controller/main.py', 'forward'])

        elif direction == 'backward':
            print("Moving backward")
            motor_process = subprocess.Popen(['sudo', '-E', 'python', 'motor_controller/main.py', 'reverse'])

        elif direction == 'stop':
            print("Stopping motor")
            if motor_process:
                motor_process.terminate()
                motor_process = None

        return jsonify({'status': 'success', 'message': f'Motor moving {direction}'})

    except Exception as e:
        print(f"Error in control_motor route: {str(e)}")  # Log any errors
        return jsonify({'status': 'error', 'message': str(e)})




# Route to start motor control using follow_pid.py
@app.route('/motor_control')
def motor_control():
    return render_template('motor_control.html')

@app.route('/click')
def click():
    Camera.release_instance()
    return render_template('index.html')

@app.route('/click_position', methods=['POST'])
def click_position():
    data = request.get_json()
    x = int(data['x'])  # Ensure x is an integer
    y = int(data['y'])  # Ensure y is an integer
    frame_width = 640  # Assuming a 640x480 resolution
    max_turn_angle = 27  # Maximum turn angle in degrees

    # Calculate the turn angle based on the x-coordinate
    center_x = frame_width / 2
    deviation = x - center_x
    normalized_deviation = deviation / center_x  # Range: -1 to 1
    turn_angle = normalized_deviation * max_turn_angle
    turn_direction = "Right" if turn_angle > 0 else "Left"

    print(f"Click position: X: {x}, Y: {y}, Turn: {turn_direction} {turn_angle:.2f} degrees")

    # Capture the current frame from the camera
    picam2 = Camera.get_instance()
    frame = picam2.capture_array()

    # Draw a red circle at the click position
    cv2.circle(frame, (x, y), 10, (0, 0, 255), 3)

    # Save the marked image
    marked_image_path = "static/clicked_image.jpg"
    cv2.imwrite(marked_image_path, frame)

    # Send the turn angle and image path back to the client
    return jsonify({
        "status": "success", 
        "x": x, 
        "y": data['y'], 
        "turn_angle": f"{turn_direction} {abs(round(turn_angle, 2))}",
        "image_path": marked_image_path
    })




@app.route('/video_feed')
def video_feed():
    Camera.release_instance()
    return render_template('video_feed.html')

@app.route('/video_feed_stream')
def video_feed_stream():
    picam2 = Camera.get_instance()
    return Response(gen(picam2), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/autonoms_drive')
def autonoms_drive():
    return render_template('autonoms_drive.html')

@app.route('/autonoms_drive_stream')
def autonoms_drive_stream():
    picam2 = Camera.get_instance()
    return Response(stream_with_context(detect_april_tag_direction(picam2)), mimetype='text/event-stream')

@app.route('/manual_drive')
def manual_drive():
    Camera.release_instance()
    return render_template('manual_drive.html')

@app.route('/manual_drive_action', methods=['POST'])
def manual_drive_action():
    data = request.get_json()
    direction = data['direction']

    # Process the direction (e.g., control motors, log, etc.)
    print(f"Direction: {direction}")

    return jsonify({"status": "success"})
# +++++++++++++++++++++++++++++++++++++++++++++++++++++ GYRO++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def quaternion_to_euler(quat_i, quat_j, quat_k, quat_real):
    """
    Convert quaternion to Euler angles.
    """
    roll = math.atan2(2 * (quat_real * quat_i + quat_j * quat_k), 1 - 2 * (quat_i**2 + quat_j**2))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (quat_real * quat_j - quat_k * quat_i))))
    yaw = math.atan2(2 * (quat_real * quat_k + quat_i * quat_j), 1 - 2 * (quat_j**2 + quat_k**2))
    return roll, pitch, yaw


def calibrate_imu():
    """
    Capture the initial IMU readings to use as calibration offsets.
    """
    global calibration_offsets, calibrated
    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    roll, pitch, yaw = quaternion_to_euler(quat_i, quat_j, quat_k, quat_real)
    calibration_offsets['roll'] = math.degrees(roll)
    calibration_offsets['pitch'] = math.degrees(pitch)
    calibration_offsets['yaw'] = math.degrees(yaw)
    calibrated = True

def update_imu_data():
    global calibrated
    error_count = 0
    max_errors = 10  # Maximum consecutive errors before taking action
    while True:
        try:
            if not calibrated:
                calibrate_imu()
            quat = bno.quaternion
            if quat is None:
                raise ValueError("Quaternion data is None")
            quat_i, quat_j, quat_k, quat_real = quat
            
            # Check for invalid data (e.g., all 0xFF indicates an error)
            if any(b == 0xFF for b in [quat_i, quat_j, quat_k, quat_real]):
                raise ValueError("Received invalid quaternion data containing 0xFF")
            
            roll, pitch, yaw = quaternion_to_euler(quat_i, quat_j, quat_k, quat_real)
            
            with imu_data_lock:
                imu_data['roll'] = math.degrees(roll) - calibration_offsets['roll']
                imu_data['pitch'] = math.degrees(pitch) - calibration_offsets['pitch']
                imu_data['yaw'] = math.degrees(yaw) - calibration_offsets['yaw']
            
            # Reset error count on successful read
            error_count = 0
        except Exception as e:
            error_count += 1
            if error_count <= max_errors:
                print(f"Exception in update_imu_data: {e}")
            elif error_count == max_errors + 1:
                print("Max IMU errors reached. Attempting to reset IMU.")
            elif error_count > max_errors:
                # Attempt to reset the IMU
                try:
                    bno = BNO08X_I2C(i2c)  # Reinitialize the IMU
                    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
                    print("IMU has been reset and reinitialized.")
                    calibrated = False  # Trigger recalibration
                    error_count = 0  # Reset error count after reset
                except Exception as reset_e:
                    print(f"Failed to reset IMU: {reset_e}")
                    # Optional: Sleep longer before retrying to prevent rapid failures
                    time.sleep(2)
        # Adjust sleep based on error state
        if error_count > max_errors:
            time.sleep(2)  # Longer sleep after attempting a reset
        else:
            time.sleep(0.05)  # Regular sleep interval


# Route to serve the gyro.html page
@app.route('/gyro')
def gyro():
    # Start the IMU data update thread
    return render_template('gyro.html')

# Route to fetch IMU data
@app.route('/imu_data')
def get_imu_data():
    with imu_data_lock:
        data = imu_data.copy()
    return jsonify(data)



def gen_face_detection(camera):
    """Video streaming generator function with face detection."""
    while True:
        try:
            frame = camera.capture_array()

            # Convert to grayscale for face detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, 1.1, 4)

            # Draw rectangles around the detected faces
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

            # Encode the frame as JPEG
            _, jpeg_frame = cv2.imencode('.jpg', frame)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg_frame.tobytes() + b'\r\n')
        except Exception as e:
            print(f"Error in gen_face_detection: {e}")
            break

@app.route('/face_detection')
def face_detection():
    return render_template('face_detection.html')

@app.route('/face_detection_stream')
def face_detection_stream():
    picam2 = Camera.get_instance()
    return Response(gen_face_detection(picam2), mimetype='multipart/x-mixed-replace; boundary=frame')

################# TURN ##################
turn_status = {
    'is_turning': False,
    'current_angle': 0,
    'target_angle': 0,
    'error': None
}
turn_lock = Lock()

@app.route('/turn')
def turn_control():
    return render_template('turn.html')

@app.route('/start_turn', methods=['POST'])
def start_turn():
    global turn_status
    
    # Ensure we're receiving JSON data
    if not request.is_json:
        return jsonify({'error': 'Content-Type must be application/json'}), 400
    
    try:
        data = request.get_json()
        if not data:
            return jsonify({'error': 'No data provided'}), 400
            
        angle = float(data.get('angle', 0))
        
        # Validate angle
        if angle <= 0 or angle >= 180:
            return jsonify({'error': 'Angle must be between 0 and 180 (exclusive)'}), 400
        
        # Check if already turning
        with turn_lock:
            if turn_status['is_turning']:
                return jsonify({'error': 'Turn already in progress'}), 400
        
        # Start turn in background thread
        thread = threading.Thread(target=run_turn_script, args=(angle,))
        thread.daemon = True  # Make thread daemon so it exits when main program exits
        thread.start()
        
        return jsonify({
            'status': 'success',
            'message': 'Turn started',
            'angle': angle
        })
    
    except ValueError:
        return jsonify({'error': 'Invalid angle value'}), 400
    except Exception as e:
        print(f"Turn error: {str(e)}")  # Log the error
        return jsonify({'error': str(e)}), 500

@app.route('/get_turn_status', methods=['GET'])
def get_turn_status():
    """Get the current status of the turn operation"""
    try:
        with turn_lock:
            status_copy = turn_status.copy()  # Make a copy to avoid lock issues
        return jsonify(status_copy)
    except Exception as e:
        print(f"Status error: {str(e)}")  # Log the error
        return jsonify({
            'is_turning': False,
            'current_angle': 0,
            'target_angle': 0,
            'error': str(e)
        })
        
def run_turn_script(angle):
    global turn_status
    with turn_lock:
        turn_status['is_turning'] = True
        turn_status['error'] = None
        turn_status['target_angle'] = angle
        turn_status['current_angle'] = 0

    try:
        script_path = os.path.join(os.path.dirname(__file__), 'turn_imu.py')
        process = subprocess.Popen(['python3', script_path, str(angle)], 
                                 stdout=subprocess.PIPE, 
                                 stderr=subprocess.PIPE,
                                 universal_newlines=True)
        
        while True:
            output = process.stdout.readline()
            if output == '' and process.poll() is not None:
                break
            if output:
                if "Current angle:" in output:
                    try:
                        current = float(output.split("Current angle:")[1].split("°")[0])
                        with turn_lock:
                            turn_status['current_angle'] = current
                    except:
                        pass

        return_code = process.poll()
        if return_code != 0:
            error_output = process.stderr.read()
            with turn_lock:
                turn_status['error'] = f"Error: {error_output}"
    
    except Exception as e:
        with turn_lock:
            turn_status['error'] = f"Error: {str(e)}"
    
    finally:
        with turn_lock:
            turn_status['is_turning'] = False

if __name__ == '__main__':
    threading.Thread(target=update_imu_data, daemon=True).start()
    app.run(host='0.0.0.0', port=5000, debug=True)
