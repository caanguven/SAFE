import RPi.GPIO as GPIO

# GPIO setup for motors
GPIO.setmode(GPIO.BOARD)

# Define pins for each motor
motor1_pins = {'forward': 7, 'backward': 26}
motor2_pins = {'forward': 22, 'backward': 29}
motor3_pins = {'forward': 32, 'backward': 11}
motor4_pins = {'forward': 13, 'backward': 12}

# List of all motors
all_motors = [motor1_pins, motor2_pins, motor3_pins, motor4_pins]

# Initialize all motor pins
for motor in all_motors:
    GPIO.setup(list(motor.values()), GPIO.OUT, initial=GPIO.LOW)

def motor_control(motor_pins, direction):
    # Ensure all motor pins are low
    stop_motor(motor_pins)
    
    # Set the selected direction pin to high
    if direction in motor_pins:
        GPIO.output(motor_pins[direction], GPIO.HIGH)

def stop_motor(motor_pins):
    # Stop the motor by setting all pins low
    GPIO.output(list(motor_pins.values()), GPIO.LOW)

def stop_all_motors():
    # Stop all motors by setting all pins low
    for motor in all_motors:
        stop_motor(motor)

def cleanup():
    # Cleanup GPIO on exit
    GPIO.cleanup()
