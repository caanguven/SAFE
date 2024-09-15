from flask import Flask, Response, request, render_template, jsonify, stream_with_context
import threading
from picamera2 import Picamera2
import cv2
import numpy as np
from pupil_apriltags import Detector
import subprocess
from multiprocessing import Queue
from multiprocessing import Process, Queue
import time

app = Flask(__name__)

command_queue = Queue()

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
            motor_process = subprocess.Popen(['sudo', '-E', 'python', 'follow_pid.py'])

        elif direction == 'backward':
            print("Moving backward")
            motor_process = subprocess.Popen(['sudo', '-E', 'python', 'follow_pid.py'])

        elif direction == 'stop':
            print("Stopping motor")
            if motor_process:
                motor_process.terminate()
                motor_process = None

        return jsonify({'status': 'success', 'message': f'Motor moving {direction}'})

    except Exception as e:
        print(f"Error in control_motor route: {str(e)}")  # Log any errors
        return jsonify({'status': 'error', 'message': str(e)})


@app.route('/keyboard_control')
def keyboard_control():
    return render_template('keyboard_control.html')

@app.route('/send_command', methods=['POST'])
def send_command():
    data = request.get_json()
    direction = data.get('direction')

    # Put the direction into the command queue
    command_queue.put(direction)
    return jsonify({'status': 'success', 'message': f'Command {direction} sent.'})


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


@app.route('/gyro')
def gyro():
    Camera.release_instance()
    return "Gyro Service - Not implemented yet"

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

def run_motor_controller(command_queue):
    # Import necessary modules for motor control
    from motor_controller import MotorController
    from pid_controller import PIDController
    from spike_filter import SpikeFilter
    from sawtooth_wave_generator import SawtoothWaveGenerator
    from adc_reader import ADCReader
    from motor import Motor

    # Initialize components
    motor = Motor()
    pid_controller = PIDController(Kp=1.0, Ki=0.1, Kd=0.05)
    adc_reader = ADCReader()
    spike_filter = SpikeFilter()
    sawtooth_generator = SawtoothWaveGenerator()
    config = {
        'dead_zone_start': 330,
        'offset': 5,
        'num_samples_for_average': 5,
        'slowdown_threshold': 20,
        'max_control_change': 5,
        'max_degrees_per_second': 60,
        'default_speed': 10
    }

    # Create a stop event for the motor control loop
    stop_event = threading.Event()

    # Initialize motor controller
    motor_controller = MotorController(
        motor=motor,
        pid_controller=pid_controller,
        adc_reader=adc_reader,
        channel=0,  # Replace with your ADC channel
        spike_filter=spike_filter,
        sawtooth_generator=sawtooth_generator,
        config=config,
        name='Motor'
    )

    # Start the motor control loop in a separate thread
    motor_thread = threading.Thread(target=motor_controller.control_loop, args=(stop_event,))
    motor_thread.start()

    try:
        while not stop_event.is_set():
            # Check for commands from the queue
            if not command_queue.empty():
                command = command_queue.get()
                print(f"Received command: {command}")
                if command == 'forward':
                    # Set motor to move forward
                    motor_controller.set_direction('forward')
                elif command == 'backward':
                    # Set motor to move backward
                    motor_controller.set_direction('backward')
                elif command == 'stop':
                    # Stop the motor
                    motor_controller.stop_motor()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Signal the motor control loop to stop
        stop_event.set()
        motor_thread.join()
        # Clean up motor GPIO
        motor_controller.motor.cleanup()

if __name__ == '__main__':
    # Start the motor control process
    motor_process = Process(target=run_motor_controller, args=(command_queue,))
    motor_process.start()

    try:
        app.run(host='0.0.0.0', port=5000, debug=True)
    finally:
        # Terminate the motor process when the Flask app stops
        motor_process.terminate()
        motor_process.join()

