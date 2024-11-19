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
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
import queue
import RPi.GPIO as GPIO
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

app = Flask(__name__)

# Global variables
motor_controller_thread = None
command_queue = queue.Queue()

# Initialize BNO08x IMU
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
bno = BNO08X_I2C(i2c)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

# Global variable for IMU data
imu_data = {'roll': 0, 'pitch': 0, 'yaw': 0}
calibration_offsets = {'roll': None, 'pitch': None, 'yaw': None}
calibrated = False

imu_data_lock = threading.Lock()

# Initialize the AprilTag detector
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
    """Video streaming generator function."""
    while True:
        frame = camera.capture_array()
        # Get the image per frame
        _, jpeg_frame = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg_frame.tobytes() + b'\r\n')

# +++++++++++++++++++++++++++++++++++++++++++++++++++++ Motor Control +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

# Constants for SPI and ADC
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023
MIN_ANGLE = 0
MAX_ANGLE = 330
SAWTOOTH_PERIOD = 2  # Period in seconds

# GPIO Pins for Motor 1
MOTOR1_IN1 = 7
MOTOR1_IN2 = 26
MOTOR1_SPD = 18
MOTOR1_ADC_CHANNEL = 0

# GPIO Pins for Motor 2
MOTOR2_IN1 = 29
MOTOR2_IN2 = 22
MOTOR2_SPD = 31
MOTOR2_ADC_CHANNEL = 1

# GPIO Pins for Motor 3
MOTOR3_IN1 = 11
MOTOR3_IN2 = 32
MOTOR3_SPD = 33
MOTOR3_ADC_CHANNEL = 2

# GPIO Pins for Motor 4
MOTOR4_IN1 = 12
MOTOR4_IN2 = 13
MOTOR4_SPD = 35
MOTOR4_ADC_CHANNEL = 3

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

    def read_position(self, mcp):
        raw_value = mcp.read_adc(self.adc_channel)
        
        if self.encoder_flipped:
            raw_value = ADC_MAX - raw_value
            
        filtered_value = self.spike_filter.filter(raw_value)
        
        if filtered_value is None:
            return self.last_valid_position if self.last_valid_position is not None else 0
            
        degrees = (filtered_value / ADC_MAX) * 330.0
        self.last_valid_position = degrees
        return degrees

    def set_motor_direction(self, direction):
        if direction == 'forward':
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)

    def move_to_position(self, target, mcp):
        current_position = self.read_position(mcp)
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

def generate_sawtooth_position(start_time, period=SAWTOOTH_PERIOD, max_angle=MAX_ANGLE):
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
            position = (base_position + self.group_phase_difference) % MAX_ANGLE
            if self.direction == -1:
                position = (MAX_ANGLE - position) % MAX_ANGLE
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
    elif direction == 'backward':
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
    elif direction == 'left':
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

class MotorControllerThread(threading.Thread):
    def __init__(self, command_queue):
        super().__init__()
        self.command_queue = command_queue
        self.running = True
        self.current_direction = 'stable'
        self.start_time = time.time()
        self.motors = {}  # Dictionary to hold motor instances
        self.initialize_motors()

    def initialize_motors(self):
        # GPIO setup
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(MOTOR1_IN1, GPIO.OUT)
        GPIO.setup(MOTOR1_IN2, GPIO.OUT)
        GPIO.setup(MOTOR1_SPD, GPIO.OUT)
        GPIO.setup(MOTOR2_IN1, GPIO.OUT)
        GPIO.setup(MOTOR2_IN2, GPIO.OUT)
        GPIO.setup(MOTOR2_SPD, GPIO.OUT)
        GPIO.setup(MOTOR3_IN1, GPIO.OUT)
        GPIO.setup(MOTOR3_IN2, GPIO.OUT)
        GPIO.setup(MOTOR3_SPD, GPIO.OUT)
        GPIO.setup(MOTOR4_IN1, GPIO.OUT)
        GPIO.setup(MOTOR4_IN2, GPIO.OUT)
        GPIO.setup(MOTOR4_SPD, GPIO.OUT)

        # Set up PWM for motor speed control
        self.motor1_pwm = GPIO.PWM(MOTOR1_SPD, 1000)
        self.motor1_pwm.start(0)
        self.motor2_pwm = GPIO.PWM(MOTOR2_SPD, 1000)
        self.motor2_pwm.start(0)
        self.motor3_pwm = GPIO.PWM(MOTOR3_SPD, 1000)
        self.motor3_pwm.start(0)
        self.motor4_pwm = GPIO.PWM(MOTOR4_SPD, 1000)
        self.motor4_pwm.start(0)

        # Set up MCP3008
        self.mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

        # Initialize motors
        motor1 = MotorController("M1", MOTOR1_IN1, MOTOR1_IN2, self.motor1_pwm,
                                MOTOR1_ADC_CHANNEL, encoder_flipped=False)
        motor2 = MotorController("M2", MOTOR2_IN1, MOTOR2_IN2, self.motor2_pwm,
                                MOTOR2_ADC_CHANNEL, encoder_flipped=True)
        motor3 = MotorController("M3", MOTOR3_IN1, MOTOR3_IN2, self.motor3_pwm,
                                MOTOR3_ADC_CHANNEL, encoder_flipped=False)
        motor4 = MotorController("M4", MOTOR4_IN1, MOTOR4_IN2, self.motor4_pwm,
                                MOTOR4_ADC_CHANNEL, encoder_flipped=True)

        self.motors = {
            'M1': motor1,
            'M2': motor2,
            'M3': motor3,
            'M4': motor4
        }

    def run(self):
        try:
            last_input_time = time.time()
            INPUT_TIMEOUT = 0.5  # Timeout in seconds

            while self.running:
                try:
                    # Non-blocking get from the queue
                    command = self.command_queue.get_nowait()
                    if command == 'quit':
                        self.running = False
                        break
                    else:
                        self.current_direction = command
                        last_input_time = time.time()
                except queue.Empty:
                    pass  # No new command, continue with current direction

                current_time = time.time()
                if current_time - last_input_time > INPUT_TIMEOUT:
                    # Auto-stop if no input received within timeout period
                    self.current_direction = 'stable'

                if self.current_direction != 'stable':
                    motor_groups = configure_motor_groups(self.current_direction, self.motors)
                    group1, group2 = motor_groups

                    base_position = generate_sawtooth_position(self.start_time)

                    group1_targets = group1.generate_target_positions(base_position)
                    group2_targets = group2.generate_target_positions(base_position)

                    for motor, target in zip(group1.motors, group1_targets):
                        motor.move_to_position(target, self.mcp)

                    for motor, target in zip(group2.motors, group2_targets):
                        motor.move_to_position(target, self.mcp)
                else:
                    for motor in self.motors.values():
                        motor.stop_motor()

                time.sleep(0.02)  # Sleep to prevent high CPU usage

        finally:
            self.cleanup()

    def cleanup(self):
        # Stop motors and clean up GPIO
        for motor in self.motors.values():
            motor.stop_motor()

        self.motor1_pwm.stop()
        self.motor2_pwm.stop()
        self.motor3_pwm.stop()
        self.motor4_pwm.stop()
        GPIO.cleanup()

# Routes to start and stop the motor controller
@app.route('/start_move_direction', methods=['POST'])
def start_move_direction():
    global motor_controller_thread
    if motor_controller_thread is None or not motor_controller_thread.is_alive():
        try:
            motor_controller_thread = MotorControllerThread(command_queue)
            motor_controller_thread.start()
            return jsonify({'status': 'success', 'message': 'Motor controller started successfully.'})
        except Exception as e:
            print(f"Error starting motor controller thread: {e}")
            return jsonify({'status': 'error', 'message': str(e)}), 500
    else:
        return jsonify({'status': 'error', 'message': 'Motor controller is already running.'}), 400

@app.route('/stop_move_direction', methods=['POST'])
def stop_move_direction():
    global motor_controller_thread
    if motor_controller_thread and motor_controller_thread.is_alive():
        try:
            motor_controller_thread.running = False
            motor_controller_thread.join()
            motor_controller_thread = None
            return '', 200
        except Exception as e:
            print(f"Error stopping motor controller thread: {e}")
            return jsonify({'status': 'error', 'message': str(e)}), 500
    return '', 200  # If the thread is already stopped or was never started

@app.route('/send_command', methods=['POST'])
def send_command():
    try:
        data = request.get_json()
        direction = data.get('direction')
        if direction:
            command_queue.put(direction)
            return jsonify({'status': 'success', 'message': f'Command {direction} sent to motor controller.'})
        else:
            return jsonify({'status': 'error', 'message': 'No direction provided.'}), 400
    except Exception as e:
        print(f"Error in send_command route: {str(e)}")
        return jsonify({'status': 'error', 'message': str(e)}), 500

# +++++++++++++++++++++++++++++++++++++++++++++++++++++ IMU +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

# Route to serve the motor_control.html page
@app.route('/motor_control')
def motor_control():
    # Start the IMU data update thread if not already running
    if not hasattr(app, 'imu_thread') or not app.imu_thread.is_alive():
        app.imu_thread = threading.Thread(target=update_imu_data, daemon=True)
        app.imu_thread.start()
    return render_template('motor_control.html')

# Route to fetch IMU data
@app.route('/imu_data')
def get_imu_data():
    with imu_data_lock:
        data = imu_data.copy()
    return jsonify(data)

# +++++++++++++++++++++++++++++++++++++++++++++++++++++ Other Routes +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

@app.route('/')
def main_page():
    return render_template('main.html')

@app.route('/video_feed_stream')
def video_feed_stream():
    picam2 = Camera.get_instance()
    return Response(gen(picam2), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
