from flask import Flask, Response, render_template, request, jsonify
from picamera2 import Picamera2, Preview
from io import BytesIO
import cv2
import numpy as np
from pupil_apriltags import Detector

app = Flask(__name__)

# Initialize the AprilTag libraries
at_detector = Detector(families='tag36h11 tag52h13',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

screen_center = (320, 240)

# Singleton pattern for Picamera2 instance
class Camera:
    instance = None

    @staticmethod
    def get_instance():
        if Camera.instance is None:
            Camera.instance = Picamera2()
            Camera.instance.configure(Camera.instance.create_preview_configuration(main={"size": (640, 480)}))
            Camera.instance.start()
        return Camera.instance

    @staticmethod
    def release_instance():
        if Camera.instance is not None:
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

@app.route('/')
def main_page():
    # Render the main.html template with buttons to other services
    return render_template('main.html')

@app.route('/click')
def click():
    Camera.release_instance()
    return render_template('index.html')

@app.route('/click_position', methods=['POST'])
def click_position():
    data = request.get_json()
    print(f"Click position: {data['x']}, {data['y']}")
    # You can process the click position here
    return jsonify({"status": "success", "x": data['x'], "y": data['y']})

@app.route('/video_feed')
def video_feed():
    Camera.release_instance()
    """Render the video_feed.html template."""
    return render_template('video_feed.html')

@app.route('/video_feed_stream')
def video_feed_stream():
    Camera.release_instance()
    """Video streaming route. Put this in the src attribute of an img tag."""
    picam2 = Camera.get_instance()
    return Response(gen(picam2),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/manual_drive')
def manual_drive():
    Camera.release_instance()
    return "Manual Drive Service - Not implemented yet"

@app.route('/autonoms_drive')
def autonoms_drive():
    Camera.release_instance()
    return "Autonomous Drive Service - Not implemented yet"

@app.route('/gyro')
def gyro():
    Camera.release_instance()
    return "Gyro Service - Not implemented yet"

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=80, debug=True)
