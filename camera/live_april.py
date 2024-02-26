from flask import Flask, Response, render_template,request, jsonify
from picamera import PiCamera
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

screen_center = (320,240)

def gen(camera):
    """Video streaming generator function with AprilTag detection."""
    while True:
        stream = BytesIO()
        camera.capture(stream, 'jpeg', use_video_port=True)
        stream.seek(0)
        frame_data = np.frombuffer(stream.getvalue(), dtype=np.uint8)
        frame = cv2.imdecode(frame_data, 1)

        # Converts to gray
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = at_detector.detect(gray, estimate_tag_pose=False, camera_params=None, tag_size=None)

        # Process
        for tag in tags:
          #corner points
          bottom_left = (int(tag.corners[0][0]), int(tag.corners[0][1]))
          top_right = (int(tag.corners[2][0]), int(tag.corners[2][1]))

          # Draw the rectangle s 
          cv2.rectangle(frame, bottom_left, top_right, (0,255,0), 2)

          #draw the tag ID 
          center = (int(tag.center[0]), int(tag.center[1]))
          cv2.putText(frame, f"ID {tag.tag_id}", center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
           
           
          tag_center = (int(tag.center[0]), int(tag.center[1]))
          
          #pose of the tag
          relative_position = (center[0] - screen_center[0], -(center[1] - screen_center[1]))

          # show relative pose 
          cv2.putText(frame, f"Rel Pos: {relative_position}", (center[0]+10, center[1]+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
         
          x_pos_relative_to_center = tag.center[0] - screen_center[0]
          mapped_x_value = (x_pos_relative_to_center / 320) * 60
          print(f"Mapped X Value: {mapped_x_value}")

        # get the image per frame
        _, jpeg_frame = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg_frame.tobytes() + b'\r\n')


@app.route('/click')
def index():
    # Render the video_stream.html template
    return render_template('index.html') 


@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen(PiCamera(resolution='640x480', framerate=60)),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/click_position', methods=['POST'])
def click_position():
    data = request.get_json()
    print(f"Click position: {data['x']}, {data['y']}")
    # You can process the click position here
    return jsonify({"status": "success", "x": data['x'], "y": data['y']})

# @app.route('/')
# def index():
#     html_content = """
#     <!DOCTYPE html>
#     <html>
#     <head>
#         <title>Video Feed</title> 
#     </head>
#     <body>
#         <h2>Live Video Feed</h2>
#         <img src="/video_feed" alt="Video Feed" style="display:block; width:640px; height:480px;">
#         <div style="text-align:center; margin-top:20px; font-size:20px;">Advanced Quadruped</div>
#     </body>
#     </html>
#     """
#     return html_content



if __name__ == '__main__':
    app.run(host='0.0.0.0', port=80, debug=True)
