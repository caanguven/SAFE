# app.py

from flask import Flask, render_template, request, jsonify
from motor_control import MotorControlSystem
import threading
import signal
import sys
import logging

app = Flask(__name__)

# Configure Logging for Flask App
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s [%(levelname)s] %(message)s',
                    handlers=[
                        logging.FileHandler("flask_app.log"),
                        logging.StreamHandler()
                    ])

# Initialize the MotorControlSystem with default mode 'normal'
motor_system = MotorControlSystem(mode='normal')

# Graceful shutdown handling
def signal_handler(sig, frame):
    logging.info('Shutting down gracefully...')
    motor_system.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

@app.route('/')
def index():
    logging.debug("Served index.html")
    return render_template('index.html')

@app.route('/control', methods=['POST'])
def control():
    try:
        data = request.json
        direction = data.get('direction', 'stable')
        if direction not in ['forward', 'backward', 'left', 'right', 'stable']:
            raise ValueError("Invalid direction received.")
        motor_system.set_direction(direction)
        logging.info(f"Control command received: direction='{direction}'")
        return jsonify({'status': 'success', 'direction': direction})
    except Exception as e:
        logging.error(f"Error in /control: {str(e)}")
        return jsonify({'status': 'error', 'message': str(e)}), 400

@app.route('/mode', methods=['POST'])
def mode():
    try:
        data = request.json
        mode = data.get('mode', 'normal')
        if mode not in ['normal', 'gallop']:
            raise ValueError("Invalid mode received.")
        motor_system.set_mode(mode)
        logging.info(f"Mode command received: mode='{mode}'")
        return jsonify({'status': 'success', 'mode': mode})
    except Exception as e:
        logging.error(f"Error in /mode: {str(e)}")
        return jsonify({'status': 'error', 'message': str(e)}), 400

@app.route('/status', methods=['GET'])
def status():
    try:
        status = motor_system.get_status()
        logging.debug("Status requested.")
        return jsonify(status)
    except Exception as e:
        logging.error(f"Error in /status: {str(e)}")
        return jsonify({'status': 'error', 'message': str(e)}), 400

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
