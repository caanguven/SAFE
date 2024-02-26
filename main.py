import RPi.GPIO as GPIO
import time
import math
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import threading
from Encoder import read_position

# Constants
Kp = 3
Ki = 0.005
Kd = 1

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)

# Motor class
class Motor:
    def __init__(self, pwm_pin, in_a, in_b, adc_channel, Kp, Ki, Kd):
        self.pwm_pin = pwm_pin
        self.in_a = in_a
        self.in_b = in_b
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.adc_channel = AnalogIn(mcp, adc_channel)
        self.setup_gpio()
        self.last_error = 0
        self.integral = 0

    def setup_gpio(self):
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        GPIO.setup(self.in_a, GPIO.OUT)
        GPIO.setup(self.in_b, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pwm_pin, 100)  # Set PWM frequency to 100 Hz
        self.pwm.start(0)

    def apply_control(self, set_position):
        current_position = read_position()
        error = set_position - current_position
        self.integral = self.integral + error * Ki
        derivative = (error - self.last_error) * Kd
        control_effort = Kp * error + self.integral + derivative
        self.last_error = error
        
        # Assuming control_effort > 0 for CW and control_effort < 0 for CCW
        if control_effort > 0:
            GPIO.output(self.in_a, GPIO.HIGH)
            GPIO.output(self.in_b, GPIO.LOW)
        else:
            GPIO.output(self.in_a, GPIO.LOW)
            GPIO.output(self.in_b, GPIO.HIGH)
        
        # Adjust PWM duty cycle according to the magnitude of control_effort
        self.pwm.ChangeDutyCycle(min(abs(control_effort), 100))

# Initialize motors with their respective GPIO pins and ADC channels
motors = [
    Motor(pwm_pin=18, in_a=23, in_b=24, adc_channel=MCP.P0, Kp=Kp, Ki=Ki, Kd=Kd),
    Motor(pwm_pin=19, in_a=6, in_b=13, adc_channel=MCP.P1, Kp=Kp, Ki=Ki, Kd=Kd),
    Motor(pwm_pin=20, in_a=5, in_b=26, adc_channel=MCP.P2, Kp=Kp, Ki=Ki, Kd=Kd),
    Motor(pwm_pin=21, in_a=22, in_b=27, adc_channel=MCP.P3, Kp=Kp, Ki=Ki, Kd=Kd)
]

def control_loop():
    while True:
        current_time = time.time() * 1000  # Convert current time to milliseconds
        for i, motor in enumerate(motors):
            set_position = sawtooth_wave(current_time + i * 100, time_for_one_cycle, amplitude)  # Offset each motor's wave for demonstration
            motor.apply_control(set_position)
        time.sleep(0.05)  # Small delay to simulate control loop timing

# Function to generate sawtooth wave value based on current time
def sawtooth_wave(time_elapsed, period, amplitude):
    return (time_elapsed % period) * (amplitude / period)

# Start the control loop in a separate thread
control_thread = threading.Thread(target=control_loop)
control_thread.daemon = True
control_thread.start()

# Keep the script running
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()
