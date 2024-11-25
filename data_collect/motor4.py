import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import csv
import os
import logging

# Configure Logging
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s [%(levelname)s] %(message)s',
                    handlers=[
                        logging.FileHandler("motor4_test.log"),
                        logging.StreamHandler()
                    ])

# Constants
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023
MIN_ANGLE = 0
MAX_ANGLE = 330
SAWTOOTH_PERIOD = 2  # Period in seconds

# Motor 4 GPIO Pins
MOTOR4_IN1 = 12
MOTOR4_IN2 = 13
MOTOR4_SPD = 35
MOTOR4_ADC_CHANNEL = 3

class SpikeFilter:
    def __init__(self):
        self.filter_active = False
        self.last_valid_reading = None

    def filter(self, new_value):
        if self.filter_active:
            if 150 <= new_value <= 700:
                return None
            else:
                self.filter_active = False
                self.last_valid_reading = new_value
                return new_value
        else:
            if self.last_valid_reading is not None and abs(self.last_valid_reading - new_value) > 300:
                self.filter_active = True
                return None
            else:
                self.last_valid_reading = new_value
                return new_value

    def reset(self):
        self.filter_active = False
        self.last_valid_reading = None

class PIDController:
    def __init__(self, Kp=0.8, Ki=0.1, Kd=0.05):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, error):
        current_time = time.time()
        delta_time = current_time - self.last_time
        if delta_time <= 0.0:
            delta_time = 0.0001

        proportional = self.Kp * error
        self.integral += error * delta_time
        integral = self.Ki * self.integral
        derivative = self.Kd * (error - self.previous_error) / delta_time

        control_signal = proportional + integral + derivative
        self.previous_error = error
        self.last_time = current_time

        return control_signal

    def reset(self):
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()

class Motor4Controller:
    def __init__(self):
        # GPIO setup
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(MOTOR4_IN1, GPIO.OUT)
        GPIO.setup(MOTOR4_IN2, GPIO.OUT)
        GPIO.setup(MOTOR4_SPD, GPIO.OUT)

        # PWM setup
        self.pwm = GPIO.PWM(MOTOR4_SPD, 1000)
        self.pwm.start(0)

        # ADC setup
        self.mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

        # Control components
        self.pid = PIDController()
        self.spike_filter = SpikeFilter()
        self.position = 0
        self.last_valid_position = None

    def read_position(self, use_filter=True):
        raw_value = self.mcp.read_adc(MOTOR4_ADC_CHANNEL)
        
        # Flip the encoder reading
        raw_value = ADC_MAX - raw_value
        
        if use_filter:
            filtered_value = self.spike_filter.filter(raw_value)
            if filtered_value is None:
                return self.last_valid_position if self.last_valid_position is not None else 0
            value_to_use = filtered_value
        else:
            value_to_use = raw_value
            
        degrees = (value_to_use / ADC_MAX) * 330.0
        self.last_valid_position = degrees
        return degrees, raw_value

    def set_motor_direction(self, direction):
        if direction == 'forward':
            GPIO.output(MOTOR4_IN1, GPIO.HIGH)
            GPIO.output(MOTOR4_IN2, GPIO.LOW)
        else:
            GPIO.output(MOTOR4_IN1, GPIO.LOW)
            GPIO.output(MOTOR4_IN2, GPIO.HIGH)

    def stop_motor(self):
        GPIO.output(MOTOR4_IN1, GPIO.LOW)
        GPIO.output(MOTOR4_IN2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)

    def cleanup(self):
        self.stop_motor()
        self.pwm.stop()
        GPIO.cleanup()

def setup_csv_logger(filename, headers):
    if not os.path.exists('logs'):
        os.makedirs('logs')
    file = open(filename, 'w', newline='')
    writer = csv.DictWriter(file, fieldnames=headers)
    writer.writeheader()
    return file, writer

def generate_sawtooth_reference(elapsed_time, period=2):
    position_in_cycle = (elapsed_time % period) / period
    return (position_in_cycle * MAX_ANGLE) % MAX_ANGLE

def run_test(duration, use_filter=True):
    motor4 = Motor4Controller()
    
    headers = ['timestamp', 'sawtooth_reference', 'position_degrees', 
              'raw_value', 'control_signal', 'error', 'spike_filter']
    
    filter_status = "with_filter" if use_filter else "without_filter"
    log_file, csv_writer = setup_csv_logger(
        f'logs/motor4_test_{filter_status}.csv', headers)
    
    try:
        print(f"Starting {duration}s test {filter_status}")
        
        # Calibration period
        print("Calibrating Motor 4...")
        time.sleep(3)
        
        start_time = time.time()
        
        while True:
            elapsed_time = time.time() - start_time
            
            if elapsed_time >= duration:
                break
            
            # Generate reference position
            reference = generate_sawtooth_reference(elapsed_time)
            
            # Read current position
            current_position, raw_value = motor4.read_position(use_filter)
            
            # Calculate error
            error = reference - current_position
            if error > 165:
                error -= 330
            elif error < -165:
                error += 330
            
            # Compute control signal
            control_signal = motor4.pid.compute(error)
            control_signal_percent = min(100, max(30, abs(control_signal)))
            
            # Set motor direction and speed
            direction = 'forward' if control_signal > 0 else 'backward'
            motor4.set_motor_direction(direction)
            motor4.pwm.ChangeDutyCycle(control_signal_percent)
            
            # Log data
            csv_writer.writerow({
                'timestamp': elapsed_time,
                'sawtooth_reference': reference,
                'position_degrees': current_position,
                'raw_value': raw_value,
                'control_signal': control_signal_percent,
                'error': error,
                'spike_filter': use_filter
            })
            
            time.sleep(0.02)  # 20ms sampling rate
            
    finally:
        motor4.stop_motor()
        log_file.close()
        print(f"Test completed {filter_status}")
        return motor4

def main():
    motor4 = None
    try:
        # Run test with spike filter (15 seconds)
        motor4 = run_test(duration=15, use_filter=True)
        
        # Small delay between tests
        time.sleep(1)
        
        # Run test without spike filter (15 seconds)
        motor4 = run_test(duration=15, use_filter=False)
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        if motor4:
            motor4.cleanup()
        print("Testing completed")

if __name__ == "__main__":
    main()