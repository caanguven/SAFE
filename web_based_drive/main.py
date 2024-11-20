# app.py

from flask import Flask, render_template, request, jsonify
from motor_control import MotorControlSystem
import threading
import signal
import sys

app = Flask(__name__)

# Initialize the MotorControlSystem with default mode 'normal'
motor_system = MotorControlSystem(mode='normal')

# Graceful shutdown handling
def signal_handler(sig, frame):
    print('Shutting down gracefully...')
    motor_system.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/control', methods=['POST'])
def control():
    data = request.json
    direction = data.get('direction', 'stable')
    motor_system.set_direction(direction)
    return jsonify({'status': 'success', 'direction': direction})

@app.route('/mode', methods=['POST'])
def mode():
    data = request.json
    mode = data.get('mode', 'normal')
    motor_system.set_mode(mode)
    return jsonify({'status': 'success', 'mode': mode})

@app.route('/status', methods=['GET'])
def status():
    status = motor_system.get_status()
    return jsonify(status)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
