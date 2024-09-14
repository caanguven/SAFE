import RPi.GPIO as GPIO

# Motor pin definitions
MOTOR_PINS = {
    1: {'IN1': 7, 'IN2': 26, 'SPD': 18, 'pwm': None},
    2: {'IN1': 22, 'IN2': 29, 'SPD': 31, 'pwm': None},
    3: {'IN1': 11, 'IN2': 32, 'SPD': 33, 'pwm': None},
    4: {'IN1': 12, 'IN2': 13, 'SPD': 35, 'pwm': None},
}

# Setup GPIO
GPIO.setmode(GPIO.BOARD)

def setup_motor_pins(motor_number):
    """Setup GPIO pins for the specified motor."""
    motor = MOTOR_PINS[motor_number]
    GPIO.setup(motor['IN1'], GPIO.OUT)
    GPIO.setup(motor['IN2'], GPIO.OUT)
    GPIO.setup(motor['SPD'], GPIO.OUT)

    # Initialize PWM only once
    if MOTOR_PINS[motor_number]['pwm'] is None:
        MOTOR_PINS[motor_number]['pwm'] = GPIO.PWM(motor['SPD'], 1000)  # 1kHz PWM frequency
        MOTOR_PINS[motor_number]['pwm'].start(0)  # Start PWM with 0% duty cycle

def control_motor(motor_number, direction, speed=100):
    """Control the motor direction and speed."""
    motor = MOTOR_PINS[motor_number]
    pwm = motor['pwm']

    if direction == "forward":
        GPIO.output(motor['IN1'], GPIO.HIGH)
        GPIO.output(motor['IN2'], GPIO.LOW)
    elif direction == "reverse":
        GPIO.output(motor['IN1'], GPIO.LOW)
        GPIO.output(motor['IN2'], GPIO.HIGH)
    elif direction == "stop":
        GPIO.output(motor['IN1'], GPIO.LOW)
        GPIO.output(motor['IN2'], GPIO.LOW)

    pwm.ChangeDutyCycle(speed)
    return pwm  # Return the PWM object
