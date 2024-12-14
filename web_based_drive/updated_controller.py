import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import threading
import logging
import numpy as np

# Configure Logging
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s [%(levelname)s] %(message)s',
                    handlers=[
                        logging.FileHandler("motor_control.log"),
                        logging.StreamHandler()
                    ])

# Constants
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023
MIN_ANGLE = 0
MAX_ANGLE = 360  # Changed to 360 as per thesis
SAWTOOTH_PERIOD = 2  # Period in seconds

# Motor GPIO Pin Configuration
MOTOR_CONFIGS = {
    'M1': {'IN1': 4, 'IN2': 7, 'SPD': 24, 'ADC': 0, 'FLIP': False},
    'M2': {'IN1': 5, 'IN2': 25, 'SPD': 6, 'ADC': 1, 'FLIP': True},
    'M3': {'IN1': 17, 'IN2': 12, 'SPD': 13, 'ADC': 2, 'FLIP': False},
    'M4': {'IN1': 18, 'IN2': 27, 'SPD': 19, 'ADC': 3, 'FLIP': True}
}

class SpikeFilter:
    def __init__(self, name, threshold=300):
        self.name = name
        self.threshold = threshold
        self.last_valid_reading = None
        self.filter_active = False
        self.dead_zone_lower = 150
        self.dead_zone_upper = 700

    def filter(self, new_value):
        # Dead zone detection
        if self.dead_zone_lower <= new_value <= self.dead_zone_upper:
            logging.debug(f"{self.name} SpikeFilter: Dead zone detected at {new_value}")
            return None

        # Spike detection
        if self.last_valid_reading is not None:
            if abs(self.last_valid_reading - new_value) > self.threshold:
                logging.debug(f"{self.name} SpikeFilter: Spike detected: {new_value}")
                return None

        self.last_valid_reading = new_value
        return new_value

    def reset(self):
        self.last_valid_reading = None
        self.filter_active = False

class PIDController:
    def __init__(self, Kp, Ki, Kd, name='PID'):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()
        self.name = name
        self.max_integral = 100  # Prevent integral windup

    def compute(self, error, dt=None):
        if dt is None:
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time

        if dt <= 0:
            dt = 0.001

        # Proportional term
        P = self.Kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
        I = self.Ki * self.integral

        # Derivative term
        D = self.Kd * (error - self.previous_error) / dt
        self.previous_error = error

        # Calculate total control signal
        control_signal = P + I + D
        
        logging.debug(f"{self.name} PID: P={P:.2f}, I={I:.2f}, D={D:.2f}, total={control_signal:.2f}")
        
        return control_signal

    def reset(self):
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()

class MotorController:
    def __init__(self, name, in1, in2, pwm, adc_channel, encoder_flipped=False):
        self.name = name
        self.in1 = in1
        self.in2 = in2
        self.pwm = pwm
        self.adc_channel = adc_channel
        self.encoder_flipped = encoder_flipped
        
        # Controller parameters
        self.pid = PIDController(Kp=0.8, Ki=0.1, Kd=0.05, name=f'PID-{name}')
        self.spike_filter = SpikeFilter(name)
        
        # Dead zone and rate limiting parameters
        self.dead_zone_active = False
        self.dead_zone_speed_limit = 50  # 50% speed limit in dead zone
        self.last_control_signal = 0
        self.max_rate_change = 10  # Maximum change in control signal per iteration
        self.slowdown_threshold = 20  # Degrees
        
        # Position tracking
        self.last_valid_position = None
        self.initial_position = None
        self.startup_complete = False
        
        # Performance monitoring
        self.performance_data = {
            'timestamps': [],
            'positions': [],
            'targets': [],
            'control_signals': []
        }

    def read_position(self, mcp):
        raw_value = mcp.read_adc(self.adc_channel)
        
        if self.encoder_flipped:
            raw_value = ADC_MAX - raw_value
            
        filtered_value = self.spike_filter.filter(raw_value)
        
        if filtered_value is None:
            self.dead_zone_active = True
            return 0 if self.last_valid_position is None else self.last_valid_position
            
        self.dead_zone_active = False
        degrees = (filtered_value / ADC_MAX) * MAX_ANGLE
        self.last_valid_position = degrees
        
        # Handle initial position reading
        if self.initial_position is None:
            self.initial_position = degrees
            
        return degrees

    def limit_rate_of_change(self, new_signal, max_change):
        change = new_signal - self.last_control_signal
        if abs(change) > max_change:
            change = max_change if change > 0 else -max_change
        return self.last_control_signal + change

    def move_to_position(self, target, mcp):
        try:
            current_time = time.time()
            current_position = self.read_position(mcp)
            
            # Smooth startup transition
            if not self.startup_complete:
                target = self.smooth_startup(target, current_position)
            
            # Calculate error
            error = target - current_position
            if abs(error) > MAX_ANGLE/2:
                error = error - np.sign(error) * MAX_ANGLE

            # Generate control signal
            control_signal = self.pid.compute(error)
            
            # Apply rate limiting
            max_change = self.max_rate_change
            if self.dead_zone_active:
                max_change /= 2
            control_signal = self.limit_rate_of_change(control_signal, max_change)
            
            # Apply speed limits and adjustments
            if self.dead_zone_active:
                control_signal = min(abs(control_signal), self.dead_zone_speed_limit) * np.sign(control_signal)
            elif abs(error) < self.slowdown_threshold:
                reduction_factor = abs(error) / self.slowdown_threshold
                control_signal *= reduction_factor

            # Set motor direction and speed
            direction = 'forward' if control_signal > 0 else 'backward'
            self.set_motor_direction(direction)
            
            speed = min(100, max(30, abs(control_signal)))
            self.pwm.ChangeDutyCycle(speed)
            
            # Store performance data
            self.performance_data['timestamps'].append(current_time)
            self.performance_data['positions'].append(current_position)
            self.performance_data['targets'].append(target)
            self.performance_data['control_signals'].append(speed)
            
            # Update control signal history
            self.last_control_signal = control_signal
            
            return abs(error) <= 2

        except Exception as e:
            logging.error(f"{self.name} Error in move_to_position: {e}")
            self.reset()
            return False

    def smooth_startup(self, target, current_position):
        if not self.startup_complete:
            if abs(target - current_position) < 5:
                self.startup_complete = True
                return target
            else:
                return current_position + np.sign(target - current_position) * 5
        return target

    def set_motor_direction(self, direction):
        if direction == 'forward':
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)

    def stop_motor(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)

    def reset(self):
        self.stop_motor()
        self.pid.reset()
        self.spike_filter.reset()
        self.last_control_signal = 0
        self.dead_zone_active = False
        self.performance_data = {
            'timestamps': [],
            'positions': [],
            'targets': [],
            'control_signals': []
        }

class MotorControlSystem:
    def __init__(self):
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        self.setup_gpio()
        
        # Initialize MCP3008
        self.mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))
        
        # Initialize motors
        self.motors = {}
        self.init_motors()
        
        # Control parameters
        self.running = True
        self.start_time = time.time()
        self.current_direction = 'stable'
        self.lock = threading.Lock()
        
        # Start control loop
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()

    def setup_gpio(self):
        for config in MOTOR_CONFIGS.values():
            GPIO.setup(config['IN1'], GPIO.OUT)
            GPIO.setup(config['IN2'], GPIO.OUT)
            GPIO.setup(config['SPD'], GPIO.OUT)

    def init_motors(self):
        for name, config in MOTOR_CONFIGS.items():
            pwm = GPIO.PWM(config['SPD'], 1000)
            pwm.start(0)
            self.motors[name] = MotorController(
                name=name,
                in1=config['IN1'],
                in2=config['IN2'],
                pwm=pwm,
                adc_channel=config['ADC'],
                encoder_flipped=config['FLIP']
            )

    def generate_sawtooth(self):
        elapsed_time = time.time() - self.start_time
        position = (elapsed_time % SAWTOOTH_PERIOD) / SAWTOOTH_PERIOD * MAX_ANGLE
        return position

    def control_loop(self):
        while self.running:
            with self.lock:
                if self.current_direction != 'stable':
                    target = self.generate_sawtooth()
                    for motor in self.motors.values():
                        motor.move_to_position(target, self.mcp)
                else:
                    for motor in self.motors.values():
                        motor.stop_motor()
            time.sleep(0.02)  # 50Hz control loop

    def set_direction(self, direction):
        with self.lock:
            self.current_direction = direction
            if direction == 'stable':
                for motor in self.motors.values():
                    motor.reset()

    def stop(self):
        self.running = False
        self.control_thread.join()
        for motor in self.motors.values():
            motor.stop_motor()
            motor.pwm.stop()
        GPIO.cleanup()

    def get_performance_data(self):
        return {name: motor.performance_data for name, motor in self.motors.items()}