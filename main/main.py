from flask import Flask, Response, request, render_template, jsonify, stream_with_context
from picamera2 import Picamera2
import cv2
import numpy as np
from pupil_apriltags import Detector
import subprocess

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

@app.route('/start_motor', methods=['POST'])
def start_motor():
    try:
        # Use sudo and -E option to run the script with the right environment and Python version
        process = subprocess.Popen(['sudo', '-E', 'python', 'follow_pid.py'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return jsonify({"status": "success", "message": "Motor control started!"}), 200
    except Exception as e:
        print(f"Error starting motor control: {e}")
        return jsonify({"status": "error", "message": str(e)}), 500


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

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=80, debug=True)
