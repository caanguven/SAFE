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

# The rest of the camera and AprilTag detection routes
@app.route('/video_feed')
def video_feed():
    Camera.release_instance()
    return render_template('video_feed.html')

@app.route('/autonoms_drive')
def autonoms_drive():
    return render_template('autonoms_drive.html')

@app.route('/autonoms_drive_stream')
def autonoms_drive_stream():
    picam2 = Camera.get_instance()
    return Response(stream_with_context(detect_april_tag_direction(picam2)), mimetype='text/event-stream')

@app.route('/motor_graph')
def motor_graph():
    return render_template('motor_graph.html')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=80, debug=True)
