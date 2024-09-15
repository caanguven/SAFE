# motor.py

import RPi.GPIO as GPIO

class Motor:
    def __init__(self, in1_pin, in2_pin, spd_pin):
        # Set GPIO mode to BCM (Broadcom SOC channel numbering)
        GPIO.setmode(GPIO.BCM)

        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.spd_pin = spd_pin

        # GPIO setup
        GPIO.setup(self.in1_pin, GPIO.OUT)
        GPIO.setup(self.in2_pin, GPIO.OUT)
        GPIO.setup(self.spd_pin, GPIO.OUT)

        # Initialize PWM on the speed pin
        self.pwm = GPIO.PWM(self.spd_pin, 1000)  # PWM at 1kHz
        self.pwm.start(0)

    def set_speed(self, speed):
        # Clamp the speed to 0-100%
        speed = max(0, min(100, speed))
        self.pwm.ChangeDutyCycle(speed)

    def forward(self):
        GPIO.output(self.in1_pin, GPIO.HIGH)
        GPIO.output(self.in2_pin, GPIO.LOW)

    def backward(self):
        GPIO.output(self.in1_pin, GPIO.LOW)
        GPIO.output(self.in2_pin, GPIO.HIGH)

    def stop(self):
        GPIO.output(self.in1_pin, GPIO.LOW)
        GPIO.output(self.in2_pin, GPIO.LOW)

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup([self.in1_pin, self.in2_pin, self.spd_pin])
