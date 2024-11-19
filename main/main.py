from flask import Flask, Response, request, render_template, jsonify, stream_with_context
from picamera2 import Picamera2
import cv2
import numpy as np
from pupil_apriltags import Detector
import threading
import time
import math
import os
import busio
import board
from threading import Lock
import RPi.GPIO as GPIO
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

app = Flask(__name__)

# Specify the path to the Haar cascade XML file
haar_cascade_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'

# Load the Haar cascade for face detection
face_cascade = cv2.CascadeClassifier(haar_cascade_path)

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

# ------------------- Motor Control Integration -------------------

# GPIO Pins and Setup
MOTOR_PINS = {
    'M1': {'IN1': 7, 'IN2': 26, 'SPD': 18, 'ADC_CHANNEL': 0, 'ENC_FLIPPED': False},
    'M2': {'IN1': 29, 'IN2': 22, 'SPD': 31, 'ADC_CHANNEL': 1, 'ENC_FLIPPED': True},
    'M3': {'IN1': 11, 'IN2': 32, 'SPD': 33, 'ADC_CHANNEL': 2, 'ENC_FLIPPED': False},
    'M4': {'IN1': 12, 'IN2': 13, 'SPD': 35, 'ADC_CHANNEL': 3, 'ENC_FLIPPED': True}
}

GPIO.setmode(GPIO.BOARD)
for motor, pins in MOTOR_PINS.items():
    GPIO.setup(pins['IN1'], GPIO.OUT)
    GPIO.setup(pins['IN2'], GPIO.OUT)
    GPIO.setup(pins['SPD'], GPIO.OUT)

# Set up PWM for motor speed control
motor_pwms = {}
for motor, pins in MOTOR_PINS.items():
    pwm = GPIO.PWM(pins['SPD'], 1000)  # 1kHz frequency
    pwm.start(0)
    motor_pwms[motor] = pwm

# Set up MCP3008
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(0, 0))

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
    def __init__(self, name, in1, in2, pwm, adc_channel, encoder_flipped=False):
        self.name = name
        self.in1 = in1
        self.in2 = in2
        self.pwm = pwm
        self.adc_channel = adc_channel
        self.position = 0
        self.last_valid_position = None
        self.pid = PIDController(Kp=0.8, Ki=0.1, Kd=0.05)
        self.encoder_flipped = encoder_flipped
        self.spike_filter = SpikeFilter(name)

    def read_position(self):
        raw_value = mcp.read_adc(self.adc_channel)
        
        if self.encoder_flipped:
            raw_value = 1023 - raw_value  # ADC_MAX = 1023
                
        filtered_value = self.spike_filter.filter(raw_value)
        
        if filtered_value is None:
            return self.last_valid_position if self.last_valid_position is not None else 0
            
        degrees = (filtered_value / 1023) * 330.0
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

        if error > 165:
            error -= 330
        elif error < -165:
            error += 330

        control_signal = self.pid.compute(error)

        if abs(error) <= 2:
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

def generate_sawtooth_position(start_time, period=2, max_angle=330):
    elapsed_time = time.time() - start_time
    position_in_cycle = (elapsed_time % period) / period
    position = (position_in_cycle * max_angle) % max_angle
    return position

class MotorGroup:
    def __init__(self, motors, group_phase_difference=0, direction=1):
        self.motors = motors
        self.group_phase_difference = group_phase_difference
        self.direction = direction

    def generate_target_positions(self, base_position):
        target_positions = []
        for motor in self.motors:
            position = (base_position + self.group_phase_difference) % 330
            if self.direction == -1:
                position = (330 - position) % 330
            target_positions.append(position)
        return target_positions

def configure_motor_groups(direction, motors):
    if direction == 'forward':
        group1 = MotorGroup(
            motors=[motors['M2'], motors['M3']],
            group_phase_difference=0,
            direction=1
        )
        group2 = MotorGroup(
            motors=[motors['M1'], motors['M4']],
            group_phase_difference=180,
            direction=1
        )
    elif direction == 'backward':  # New backward direction
        group1 = MotorGroup(
            motors=[motors['M2'], motors['M3']],
            group_phase_difference=0,
            direction=-1
        )
        group2 = MotorGroup(
            motors=[motors['M1'], motors['M4']],
            group_phase_difference=180,
            direction=-1
        )
    elif direction == 'right':
        group1 = MotorGroup(
            motors=[motors['M1'], motors['M3']],
            group_phase_difference=0,
            direction=-1
        )
        group2 = MotorGroup(
            motors=[motors['M2'], motors['M4']],
            group_phase_difference=180,
            direction=1
        )
    elif direction == 'left':  # New left direction
        group1 = MotorGroup(
            motors=[motors['M1'], motors['M3']],
            group_phase_difference=0,
            direction=1
        )
        group2 = MotorGroup(
            motors=[motors['M2'], motors['M4']],
            group_phase_difference=180,
            direction=-1
        )
    else:
        raise ValueError("Invalid direction")
    return [group1, group2]

# Initialize Motors
motor1 = MotorController("M1", MOTOR_PINS['M1']['IN1'], MOTOR_PINS['M1']['IN2'], motor_pwms['M1'],
                        MOTOR_PINS['M1']['ADC_CHANNEL'], encoder_flipped=MOTOR_PINS['M1']['ENC_FLIPPED'])
motor2 = MotorController("M2", MOTOR_PINS['M2']['IN1'], MOTOR_PINS['M2']['IN2'], motor_pwms['M2'],
                        MOTOR_PINS['M2']['ADC_CHANNEL'], encoder_flipped=MOTOR_PINS['M2']['ENC_FLIPPED'])
motor3 = MotorController("M3", MOTOR_PINS['M3']['IN1'], MOTOR_PINS['M3']['IN2'], motor_pwms['M3'],
                        MOTOR_PINS['M3']['ADC_CHANNEL'], encoder_flipped=MOTOR_PINS['M3']['ENC_FLIPPED'])
motor4 = MotorController("M4", MOTOR_PINS['M4']['IN1'], MOTOR_PINS['M4']['IN2'], motor_pwms['M4'],
                        MOTOR_PINS['M4']['ADC_CHANNEL'], encoder_flipped=MOTOR_PINS['M4']['ENC_FLIPPED'])

motors = {
    'M1': motor1,
    'M2': motor2,
    'M3': motor3,
    'M4': motor4
}

# ------------------- Motor Control Routes -------------------

@app.route('/control_motor', methods=['POST'])
def control_motor():
    data = request.get_json()
    direction = data.get('direction')
    gait = data.get('gait', 'walk')  # Default gait is 'walk'

    if direction not in ['forward', 'backward', 'left', 'right', 'stop']:
        return jsonify({'status': 'error', 'message': 'Invalid direction'}), 400

    if direction == 'stop':
        for motor in motors.values():
            motor.stop_motor()
        return jsonify({'status': 'success', 'message': 'Motors stopped'})
    
    # Configure motor groups based on direction and gait if needed
    # For simplicity, gait is not used here, but you can implement different gait patterns
    try:
        motor_groups = configure_motor_groups(direction, motors)
        group1, group2 = motor_groups

        # Base position can be adjusted based on gait
        start_time = time.time()

        # This example uses a single step; for continuous movement, consider threading or asynchronous execution
        base_position = generate_sawtooth_position(start_time)

        group1_targets = group1.generate_target_positions(base_position)
        group2_targets = group2.generate_target_positions(base_position)

        for motor, target in zip(group1.motors, group1_targets):
            motor.move_to_position(target)

        for motor, target in zip(group2.motors, group2_targets):
            motor.move_to_position(target)

        return jsonify({'status': 'success', 'message': f'Motors moving {direction}'})

    except Exception as e:
        print(f"Error in control_motor route: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 500

# ------------------- Existing Routes -------------------

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
    # Start the IMU data update thread if not already running
    if not hasattr(app, 'imu_thread') or not app.imu_thread.is_alive():
        app.imu_thread = threading.Thread(target=update_imu_data, daemon=True)
        app.imu_thread.start()
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
    
    if not request.is_json:
        return jsonify({'error': 'Content-Type must be application/json'}), 400
    
    try:
        data = request.get_json()
        if not data:
            return jsonify({'error': 'No data provided'}), 400
            
        angle = float(data.get('angle', 0))
        
        # Validate angle (-180 to 180, excluding both -180 and 180)
        if angle >= 180 or angle <= -180 or angle == 0:
            return jsonify({'error': 'Angle must be between -180° and 180° (exclusive) and not 0°'}), 400
        
        # Check if already turning
        with turn_lock:
            if turn_status['is_turning']:
                return jsonify({'error': 'Turn already in progress'}), 400
        
        # Start turn in background thread
        thread = threading.Thread(target=run_turn_script, args=(angle,))
        thread.daemon = True
        thread.start()
        
        return jsonify({
            'status': 'success',
            'message': f"Starting {'right' if angle > 0 else 'left'} turn of {abs(angle)}°",
            'angle': angle
        })
    
    except ValueError:
        return jsonify({'error': 'Invalid angle value'}), 400
    except Exception as e:
        print(f"Turn error: {str(e)}")
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
        # Here, integrate your turning logic directly instead of calling an external script
        # Example: Implement turning based on angle
        # This is a placeholder for your actual motor control logic
        # Replace this with your specific turning implementation

        # Example logic:
        target_angle = angle
        while True:
            # Read current positions
            m1_pos = motors['M1'].read_position()
            m2_pos = motors['M2'].read_position()
            m3_pos = motors['M3'].read_position()
            m4_pos = motors['M4'].read_position()

            # Calculate errors and control signals
            # Implement your PID control here based on the target_angle

            # Placeholder: Check if target is reached
            # Replace with actual condition
            if abs(m1_pos - target_angle) < 2 and abs(m2_pos - target_angle) < 2:
                break

            # Update motor positions
            # Replace with actual movement commands
            # For example:
            # motors['M1'].move_to_position(target_angle)
            # motors['M2'].move_to_position(target_angle)
            time.sleep(0.1)  # Adjust sleep as needed

        # After reaching the target
        with turn_lock:
            turn_status['current_angle'] = target_angle

    except Exception as e:
        with turn_lock:
            turn_status['error'] = f"Error: {str(e)}"

    finally:
        with turn_lock:
            turn_status['is_turning'] = False

# ------------------- IMU Data Update Thread -------------------

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

# ------------------- Start IMU Data Update Thread -------------------

@app.before_first_request
def activate_job():
    """Start the IMU data update thread before handling the first request."""
    thread = threading.Thread(target=update_imu_data, daemon=True)
    thread.start()

# ------------------- Cleanup on Shutdown -------------------

@app.route('/shutdown', methods=['POST'])
def shutdown():
    """Endpoint to gracefully shutdown the server."""
    shutdown_func = request.environ.get('werkzeug.server.shutdown')
    if shutdown_func is None:
        raise RuntimeError('Not running with the Werkzeug Server')
    shutdown_func()
    return 'Server shutting down...'

# ------------------- Run the Flask App -------------------

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, debug=True)
    finally:
        # Stop all motors on shutdown
        for motor in motors.values():
            motor.stop_motor()

        # Stop all PWM channels
        for pwm in motor_pwms.values():
            pwm.stop()

        GPIO.cleanup()
