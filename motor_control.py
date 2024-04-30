import RPi.GPIO as GPIO

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

def stop_all_motors():
    for motor in all_motors:
        stop_motor(motor)

def cleanup():
    GPIO.cleanup()

