# Test script for a single motor
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR1_IN1, GPIO.OUT)
GPIO.setup(MOTOR1_IN2, GPIO.OUT)
GPIO.setup(MOTOR1_SPD, GPIO.OUT)

pwm = GPIO.PWM(MOTOR1_SPD, 100)  # Try 100Hz instead of 1000Hz
pwm.start(0)

# Test forward
GPIO.output(MOTOR1_IN1, GPIO.HIGH)
GPIO.output(MOTOR1_IN2, GPIO.LOW)
pwm.ChangeDutyCycle(80)  # Try higher speed
time.sleep(2)

# Stop
pwm.stop()
GPIO.cleanup()