import RPi.GPIO as GPIO

class Motor:
    def __init__(self, in1_pin, in2_pin, spd_pin, pwm_frequency=1000):
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.spd_pin = spd_pin
        self.pwm_frequency = pwm_frequency

        # Setup GPIO pins
        GPIO.setup(self.in1_pin, GPIO.OUT)
        GPIO.setup(self.in2_pin, GPIO.OUT)
        GPIO.setup(self.spd_pin, GPIO.OUT)

        # Setup PWM
        self.pwm = GPIO.PWM(self.spd_pin, self.pwm_frequency)
        self.pwm.start(0)  # Start with 0% duty cycle (motor off)

    def set_speed(self, duty_cycle):
        # Clamp duty_cycle to 0-100%
        duty_cycle = max(-100, min(100, duty_cycle))
        self.pwm.ChangeDutyCycle(duty_cycle)

    def forward(self):
        GPIO.output(self.in1_pin, GPIO.LOW)
        GPIO.output(self.in2_pin, GPIO.HIGH)

    def reverse(self):
        GPIO.output(self.in1_pin, GPIO.HIGH)
        GPIO.output(self.in2_pin, GPIO.LOW)

    def stop(self):
        GPIO.output(self.in1_pin, GPIO.LOW)
        GPIO.output(self.in2_pin, GPIO.LOW)
        self.set_speed(0)

    def cleanup(self):
        self.pwm.stop()
