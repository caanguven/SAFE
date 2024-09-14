import RPi.GPIO as GPIO

# Motor pin definitions
MOTOR_PINS = {
    1: {'IN1': 7, 'IN2': 26, 'SPD': 18},
    2: {'IN1': 22, 'IN2': 29, 'SPD': 31},
    3: {'IN1': 11, 'IN2': 32, 'SPD': 33},
    4: {'IN1': 12, 'IN2': 13, 'SPD': 35},
}

# Setup GPIO
GPIO.setmode(GPIO.BOARD)

def setup_motor_pins(motor_number):
    """Setup GPIO pins for the specified motor."""
    motor = MOTOR_PINS[motor_number]
    GPIO.setup(motor['IN1'], GPIO.OUT)
    GPIO.setup(motor['IN2'], GPIO.OUT)
    GPIO.setup(motor['SPD'], GPIO.OUT)

def control_motor(motor_number, direction, speed=100):
    """Control the motor direction and speed."""
    motor = MOTOR_PINS[motor_number]
    pwm = GPIO.PWM(motor['SPD'], 1000)  # 1kHz PWM frequency
    pwm.start(0)  # Start PWM with 0% duty cycle

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

    return pwm  # Return the PWM object so we can stop it later
