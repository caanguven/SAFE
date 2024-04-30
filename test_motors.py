import motor_control
import time

try:
    print("Testing motor forward.")
    motor_control.motor_control(motor_control.motor4_pins, 'forward')
    time.sleep(2)
    print("Stopping motor.")
    motor_control.stop_all_motors()
finally:
    motor_control.cleanup()
