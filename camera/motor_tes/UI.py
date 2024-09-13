from flask import Flask, Response, request, render_template, jsonify, stream_with_context
from threading import Thread
import time
import RPi.GPIO as GPIO
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import cv2
import numpy as np
from pupil_apriltags import Detector

app = Flask(__name__)

# Motor Control Setup (based on your existing motor control code)
# Pin Definitions
M4_IN1 = 12
M4_IN2 = 13
M4_SPD = 35

# MCP3008 setup
SPI_PORT = 0
SPI_DEVICE = 0
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

# Setup GPIO
GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
GPIO.setup(M4_IN1, GPIO.OUT)
GPIO.setup(M4_IN2, GPIO.OUT)
GPIO.setup(M4_SPD, GPIO.OUT)

# Setup PWM for motor speed control on M4_SPD
pwm = GPIO.PWM(M4_SPD, 1000)  # Set PWM frequency to 1kHz
pwm.start(0)  # Start with 0% duty cycle (motor off)

# PID constants
Kp = 0.1
Ki = 0.01
Kd = 0.02
OFFSET = 5
previous_error = 0
integral = 0
last_time = time.time()

# Variables for motor data logging
motor_data = []

# Function to run the motor control logic in a separate thread
def motor_controller_thread():
    global previous_error, integral, last_time, motor_data
    while True:
        pot_value = mcp.read_adc(3)  # Reading from MCP3008 ADC
        degrees_value = map_potentiometer_value_to_degrees(pot_value)
        set_position = 180  # Example static set position
        
        # PID logic here (simplified for this example)
        error = set_position - degrees_value
        integral += error
        derivative = (error - previous_error) / (time.time() - last_time)
        control_signal = Kp * error + Ki * integral + Kd * derivative
        control_signal = max(0, min(100, control_signal))
        
        # Motor control
        GPIO.output(M4_IN1, GPIO.LOW)
        GPIO.output(M4_IN2, GPIO.HIGH)
        pwm.ChangeDutyCycle(control_signal)
        
        # Store motor data for live graphing
        motor_data.append({
            'time': time.time(),
            'pot_value': degrees_value,
            'set_position': set_position,
            'control_signal': control_signal,
        })
        
        previous_error = error
        last_time = time.time()
        time.sleep(0.1)  # Adjust frequency as needed

# Start motor controller in a background thread
Thread(target=motor_controller_thread, daemon=True).start()

# Function to map potentiometer value to degrees (0 to 360)
def map_potentiometer_value_to_degrees(value):
    return value * (360 / 1023)

# Live graph data using Server-Sent Events (SSE)
@app.route('/motor_data_stream')
def motor_data_stream():
    def generate():
        while True:
            if motor_data:
                latest_data = motor_data[-1]
                yield f"data: {latest_data}\n\n"
            time.sleep(1)  # Stream data every second

    return Response(stream_with_context(generate()), mimetype='text/event-stream')

@app.route('/')
def main_page():
    return render_template('main.html')

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
