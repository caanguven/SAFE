import RPi.GPIO as GPIO
import time
import logging

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("test_motors.log"),
        logging.StreamHandler()
    ]
)

# Suppress GPIO warnings
GPIO.setwarnings(False)

# GPIO setup
GPIO.setmode(GPIO.BCM)

# Define motor pins (Replace with your actual BCM pin numbers)
MOTOR_IN1 = 4
MOTOR_IN2 = 7
MOTOR_SPD = 24

# Setup GPIO pins
GPIO.setup(MOTOR_IN1, GPIO.OUT)
GPIO.setup(MOTOR_IN2, GPIO.OUT)
GPIO.setup(MOTOR_SPD, GPIO.OUT)

# Initialize PWM
pwm = GPIO.PWM(MOTOR_SPD, 1000)  # 1kHz
pwm.start(0)

def move_forward(speed, duration):
    logging.info(f"Moving forward at {speed}% speed for {duration} seconds.")
    GPIO.output(MOTOR_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(speed)
    time.sleep(duration)
    stop_motor()

def move_backward(speed, duration):
    logging.info(f"Moving backward at {speed}% speed for {duration} seconds.")
    GPIO.output(MOTOR_IN1, GPIO.LOW)
    GPIO.output(MOTOR_IN2, GPIO.HIGH)
    pwm.ChangeDutyCycle(speed)
    time.sleep(duration)
    stop_motor()

def stop_motor():
    logging.info("Stopping motor.")
    GPIO.output(MOTOR_IN1, GPIO.LOW)
    GPIO.output(MOTOR_IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(0)
    time.sleep(1)

def main():
    try:
        move_forward(50, 2)    # Move forward at 50% speed for 2 seconds
        move_backward(50, 2)   # Move backward at 50% speed for 2 seconds
    except Exception as e:
        logging.exception("An error occurred during motor testing:")
    finally:
        logging.info("Cleaning up GPIO.")
        pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
