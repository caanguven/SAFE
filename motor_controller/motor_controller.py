import RPi.GPIO as GPIO

class MotorController:
    def __init__(self, in1, in2, spd):
        self.in1 = in1
        self.in2 = in2
        self.spd = spd
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.spd, GPIO.OUT)
        self.pwm = GPIO.PWM(self.spd, 1000)
        self.pwm.start(0)

    def set_speed(self, speed):
        self.pwm.ChangeDutyCycle(speed)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)

    def stop(self):
        self.pwm.ChangeDutyCycle(0)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup()