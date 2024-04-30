import threading
import time
from motor_control import *
from adc_reader import *
import sys
import termios
import atexit
from select import select

# Restore the terminal
atexit.register(termios.tcsetattr, sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))

def is_data():
    return select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

old_settings = termios.tcgetattr(sys.stdin)
new_settings = old_settings[:]
new_settings[3] &= ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(sys.stdin, termios.TCSADRAIN, new_settings)

def motor_thread():
    try:
        while True:
            key = getch()  # non-blocking get keyboard input
            if key == 's':
                print('s')
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

def getch():
    if is_data():
        return sys.stdin.read(1)
    return None

def adc_thread():
    try:
        while True:
            values = read_adc_values()
            print('ADC Values:', values)
            time.sleep(0.1)
    except KeyboardInterrupt:
        motor_control.cleanup()

# Start threads
thread1 = threading.Thread(target=motor_thread)
thread2 = threading.Thread(target=adc_thread)
thread1.start()
thread2.start()

# Wait for threads to complete
thread1.join()
thread2.join()
