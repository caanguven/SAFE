
import RPi.GPIO as GPIO
import time
import threading
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

# Try using a more reliable method for keyboard input in a threaded environment
import sys
import termios
import atexit
from select import select

# Restore the terminal settings when the program exits
atexit.register(termios.tcsetattr, sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))

def is_data():
    return select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

old_settings = termios.tcgetattr(sys.stdin)
new_settings = old_settings[:]
new_settings[3] &= ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(sys.stdin, termios.TCSADRAIN, new_settings)

def getch():
    if is_data():
        return sys.stdin.read(1)
    return None

# GPIO setup for motors
GPIO.setmode(GPIO.BOARD)
motor1_pins = {'forward': 7, 'backward': 26}
motor2_pins = {'forward': 22, 'backward': 29}
motor3_pins = {'forward': 32, 'backward': 11}
motor4_pins = {'forward': 13, 'backward': 12}

all_motors = [motor1_pins, motor2_pins, motor3_pins, motor4_pins]
for motor in all_motors:
    GPIO.setup(list(motor.values()), GPIO.OUT, initial=GPIO.LOW)

def motor_control(motor_pins, direction):
    GPIO.output(list(motor_pins.values()), GPIO.LOW)
    if direction in motor_pins:
        GPIO.output(motor_pins[direction], GPIO.HIGH)

def stop_motor(motor_pins):
    GPIO.output(list(motor_pins.values()), GPIO.LOW)

def motor_thread():
    try:
        while True:
            key = getch()  # non-blocking get keyboard input
            if key == 's':
                motor_control(motor1_pins, 'forward')
                motor_control(motor2_pins, 'backward')
                motor_control(motor3_pins, 'backward')
                motor_control(motor4_pins, 'backward')
            elif key == 't':
                for motor in all_motors:
                    stop_motor(motor)
            time.sleep(0.1)
    except Exception as e:
        print("Motor thread error:", e)
        GPIO.cleanup()

# MCP3008 setup
SPI_PORT = 0
SPI_DEVICE = 0
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

# Start motor control thread
thread = threading.Thread(target=motor_thread)
thread.start()

print('Reading MCP3008 values, press Ctrl-C to quit...')
print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*range(8)))
print('-' * 57)

try:
    while True:
        # Read and print ADC values continuously
        values = [mcp.read_adc(i) for i in range(8)]
        print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*values))
        time.sleep(0.02)
except KeyboardInterrupt:
    GPIO.cleanup()
    print("Stopped by User")

GPIO.cleanup()
