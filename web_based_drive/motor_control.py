# # motor_control.py

# import RPi.GPIO as GPIO
# import time
# import Adafruit_GPIO.SPI as SPI
# import Adafruit_MCP3008
# import threading
# import logging

# # Configure Logging
# logging.basicConfig(level=logging.DEBUG,
#                     format='%(asctime)s [%(levelname)s] %(message)s',
#                     handlers=[
#                         logging.FileHandler("motor_control.log"),
#                         logging.StreamHandler()
#                     ])

# # Constants for SPI and ADC
# SPI_PORT = 0
# SPI_DEVICE = 0
# ADC_MAX = 1023
# MIN_ANGLE = 0
# MAX_ANGLE = 330
# SAWTOOTH_PERIOD = 2  # Period in seconds

# # GPIO Pins for Motor 1
# MOTOR1_IN1 = 7
# MOTOR1_IN2 = 26
# MOTOR1_SPD = 18
# MOTOR1_ADC_CHANNEL = 0

# # GPIO Pins for Motor 2
# MOTOR2_IN1 = 29
# MOTOR2_IN2 = 22
# MOTOR2_SPD = 31
# MOTOR2_ADC_CHANNEL = 1

# # GPIO Pins for Motor 3
# MOTOR3_IN1 = 11
# MOTOR3_IN2 = 32
# MOTOR3_SPD = 33
# MOTOR3_ADC_CHANNEL = 2

# # GPIO Pins for Motor 4
# MOTOR4_IN1 = 12
# MOTOR4_IN2 = 13
# MOTOR4_SPD = 35
# MOTOR4_ADC_CHANNEL = 3

# class SpikeFilter:
#     def __init__(self, name):
#         self.filter_active = False
#         self.last_valid_reading = None
#         self.name = name

#     def filter(self, new_value):
#         if self.filter_active:
#             if 150 <= new_value <= 700:
#                 return None
#             else:
#                 self.filter_active = False
#                 self.last_valid_reading = new_value
#                 logging.debug(f"{self.name} SpikeFilter: Filter deactivated with value {new_value}")
#                 return new_value
#         else:
#             if self.last_valid_reading is not None and abs(self.last_valid_reading - new_value) > 300:
#                 self.filter_active = True
#                 logging.debug(f"{self.name} SpikeFilter: Spike detected with value {new_value}")
#                 return None
#             else:
#                 self.last_valid_reading = new_value
#                 return new_value

#     def reset(self):
#         self.filter_active = False
#         self.last_valid_reading = None
#         logging.debug(f"{self.name} SpikeFilter: Reset complete.")

# class PIDController:
#     def __init__(self, Kp, Ki, Kd, name='PID'):
#         self.Kp = Kp
#         self.Ki = Ki
#         self.Kd = Kd
#         self.previous_error = 0
#         self.integral = 0
#         self.last_time = time.time()
#         self.name = name

#     def compute(self, error):
#         current_time = time.time()
#         delta_time = current_time - self.last_time
#         if delta_time <= 0.0:
#             delta_time = 0.0001

#         proportional = self.Kp * error
#         self.integral += error * delta_time
#         integral = self.Ki * self.integral
#         derivative = self.Kd * (error - self.previous_error) / delta_time

#         control_signal = proportional + integral + derivative
#         self.previous_error = error
#         self.last_time = current_time

#         logging.debug(f"{self.name} PID: error={error}, proportional={proportional}, integral={integral}, derivative={derivative}, control_signal={control_signal}")

#         return control_signal

#     def reset(self):
#         self.previous_error = 0
#         self.integral = 0
#         self.last_time = time.time()
#         logging.debug(f"{self.name} PID: Reset complete.")

# class MotorController:
#     def __init__(self, name, in1, in2, pwm, adc_channel, encoder_flipped=False):
#         self.name = name
#         self.in1 = in1
#         self.in2 = in2
#         self.pwm = pwm
#         self.adc_channel = adc_channel
#         self.position = 0
#         self.last_valid_position = None
#         self.pid = PIDController(Kp=0.8, Ki=0.1, Kd=0.05, name=f'PID-{self.name}')
#         self.encoder_flipped = encoder_flipped
#         self.spike_filter = SpikeFilter(name)

#     def read_position(self, mcp):
#         raw_value = mcp.read_adc(self.adc_channel)
        
#         if self.encoder_flipped:
#             raw_value = ADC_MAX - raw_value
        
#         filtered_value = self.spike_filter.filter(raw_value)
        
#         if filtered_value is None:
#             return self.last_valid_position if self.last_valid_position is not None else 0
            
#         degrees = (filtered_value / ADC_MAX) * 330.0
#         self.last_valid_position = degrees
#         logging.debug(f"{self.name} Position: {degrees}° (raw: {raw_value})")
#         return degrees

#     def set_motor_direction(self, direction):
#         if direction == 'forward':
#             GPIO.output(self.in1, GPIO.HIGH)
#             GPIO.output(self.in2, GPIO.LOW)
#             logging.debug(f"{self.name} Direction: Forward")
#         else:
#             GPIO.output(self.in1, GPIO.LOW)
#             GPIO.output(self.in2, GPIO.HIGH)
#             logging.debug(f"{self.name} Direction: Backward")

#     def move_to_position(self, target, mcp):
#         try:
#             current_position = self.read_position(mcp)
#             error = target - current_position

#             if error > 165:
#                 error -= 330
#             elif error < -165:
#                 error += 330

#             control_signal = self.pid.compute(error)

#             if abs(error) <= 2:
#                 self.stop_motor()
#                 logging.debug(f"{self.name} reached target. Stopping motor.")
#                 return True

#             direction = 'forward' if control_signal > 0 else 'backward'
#             self.set_motor_direction(direction)
#             speed = min(100, max(30, abs(control_signal)))
#             self.pwm.ChangeDutyCycle(speed)
#             logging.debug(f"{self.name} Speed: {speed}%")
#             return False
#         except Exception as e:
#             logging.error(f"{self.name} MotorController: Error during move_to_position: {e}")
#             self.reset()
#             return False

#     def stop_motor(self):
#         GPIO.output(self.in1, GPIO.LOW)
#         GPIO.output(self.in2, GPIO.LOW)
#         self.pwm.ChangeDutyCycle(0)
#         logging.debug(f"{self.name} Motor Stopped")

#     def reset(self):
#         self.stop_motor()
#         self.pid.reset()
#         self.spike_filter.reset()
#         logging.info(f"{self.name} MotorController: Reset complete.")

# class MotorGroup:
#     def __init__(self, motors, group_phase_difference=0, direction=1):
#         self.motors = motors
#         self.group_phase_difference = group_phase_difference
#         self.direction = direction

#     def generate_target_positions(self, base_position):
#         target_positions = []
#         for motor in self.motors:
#             position = (base_position + self.group_phase_difference) % MAX_ANGLE
#             if self.direction == -1:
#                 position = (MAX_ANGLE - position) % MAX_ANGLE
#             target_positions.append(position)
#         return target_positions

# class MotorControlSystem:
#     def __init__(self, mode='normal'):
#         self.mode = mode  # 'normal' or 'gallop'
#         self.current_direction = 'stable'
#         self.lock = threading.Lock()
#         self.running = True

#         # GPIO setup
#         GPIO.setmode(GPIO.BOARD)
#         GPIO.setup(MOTOR1_IN1, GPIO.OUT)
#         GPIO.setup(MOTOR1_IN2, GPIO.OUT)
#         GPIO.setup(MOTOR1_SPD, GPIO.OUT)
#         GPIO.setup(MOTOR2_IN1, GPIO.OUT)
#         GPIO.setup(MOTOR2_IN2, GPIO.OUT)
#         GPIO.setup(MOTOR2_SPD, GPIO.OUT)
#         GPIO.setup(MOTOR3_IN1, GPIO.OUT)
#         GPIO.setup(MOTOR3_IN2, GPIO.OUT)
#         GPIO.setup(MOTOR3_SPD, GPIO.OUT)
#         GPIO.setup(MOTOR4_IN1, GPIO.OUT)
#         GPIO.setup(MOTOR4_IN2, GPIO.OUT)
#         GPIO.setup(MOTOR4_SPD, GPIO.OUT)

#         # Set up PWM for motor speed control
#         self.motor1_pwm = GPIO.PWM(MOTOR1_SPD, 1000)
#         self.motor1_pwm.start(0)
#         self.motor2_pwm = GPIO.PWM(MOTOR2_SPD, 1000)
#         self.motor2_pwm.start(0)
#         self.motor3_pwm = GPIO.PWM(MOTOR3_SPD, 1000)
#         self.motor3_pwm.start(0)
#         self.motor4_pwm = GPIO.PWM(MOTOR4_SPD, 1000)
#         self.motor4_pwm.start(0)

#         # Set up MCP3008
#         self.mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

#         # Initialize motors
#         self.motor1 = MotorController("M1", MOTOR1_IN1, MOTOR1_IN2, self.motor1_pwm,
#                                       MOTOR1_ADC_CHANNEL, encoder_flipped=False)
#         self.motor2 = MotorController("M2", MOTOR2_IN1, MOTOR2_IN2, self.motor2_pwm,
#                                       MOTOR2_ADC_CHANNEL, encoder_flipped=True)
#         self.motor3 = MotorController("M3", MOTOR3_IN1, MOTOR3_IN2, self.motor3_pwm,
#                                       MOTOR3_ADC_CHANNEL, encoder_flipped=False)
#         self.motor4 = MotorController("M4", MOTOR4_IN1, MOTOR4_IN2, self.motor4_pwm,
#                                       MOTOR4_ADC_CHANNEL, encoder_flipped=True)

#         self.motors = {
#             'M1': self.motor1,
#             'M2': self.motor2,
#             'M3': self.motor3,
#             'M4': self.motor4
#         }

#         self.start_time = time.time()

#         # Start the control loop in a separate thread
#         self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
#         self.control_thread.start()

#         logging.info(f"MotorControlSystem initialized in '{self.mode}' mode.")

#     def set_direction(self, direction):
#         with self.lock:
#             self.current_direction = direction
#             logging.info(f"Direction set to '{direction}'.")
#             if direction == 'stable':
#                 logging.info("Resetting all motors to 'stable' state.")
#                 for motor in self.motors.values():
#                     motor.reset()

#     def set_mode(self, mode):
#         with self.lock:
#             self.mode = mode
#             logging.info(f"Mode set to '{mode}'.")

#     def get_status(self):
#         with self.lock:
#             status = {
#                 'mode': self.mode,
#                 'direction': self.current_direction,
#                 'motors': {}
#             }

#             for name, motor in self.motors.items():
#                 pos = motor.read_position(self.mcp)
#                 status['motors'][name] = pos

#             # Calculate phase differences
#             m1_pos = status['motors']['M1']
#             m2_pos = status['motors']['M2']
#             m3_pos = status['motors']['M3']
#             m4_pos = status['motors']['M4']

#             phase_diff_m1_m3 = abs(m1_pos - m3_pos)
#             phase_diff_m2_m4 = abs(m2_pos - m4_pos)

#             phase_diff_m1_m3 = min(phase_diff_m1_m3, MAX_ANGLE - phase_diff_m1_m3)
#             phase_diff_m2_m4 = min(phase_diff_m2_m4, MAX_ANGLE - phase_diff_m2_m4)

#             status['phase_diff'] = {
#                 'M1-M3': phase_diff_m1_m3,
#                 'M2-M4': phase_diff_m2_m4
#             }

#             return status

#     def generate_sawtooth_position(self):
#         elapsed_time = time.time() - self.start_time
#         position_in_cycle = (elapsed_time % SAWTOOTH_PERIOD) / SAWTOOTH_PERIOD
#         position = (position_in_cycle * MAX_ANGLE) % MAX_ANGLE
#         return position

#     def configure_motor_groups(self, direction, mode):
#         if mode == 'gallop' and direction in ['forward', 'backward']:
#             # In gallop mode, M4 and M3 are synced with 180-degree phase difference to M2 and M1
#             group1 = MotorGroup(
#                 motors=[self.motors['M2'], self.motors['M1']],
#                 group_phase_difference=0,
#                 direction=1 if direction == 'forward' else -1
#             )
#             group2 = MotorGroup(
#                 motors=[self.motors['M4'], self.motors['M3']],
#                 group_phase_difference=180,
#                 direction=1 if direction == 'forward' else -1
#             )
#             logging.debug(f"Configured motor groups in 'gallop' mode for direction '{direction}'.")
#         else:
#             # Normal mode or turning commands remain unchanged
#             if direction == 'forward':
#                 group1 = MotorGroup(
#                     motors=[self.motors['M2'], self.motors['M3']],
#                     group_phase_difference=0,
#                     direction=1
#                 )
#                 group2 = MotorGroup(
#                     motors=[self.motors['M1'], self.motors['M4']],
#                     group_phase_difference=180,
#                     direction=1
#                 )
#             elif direction == 'backward':
#                 group1 = MotorGroup(
#                     motors=[self.motors['M2'], self.motors['M3']],
#                     group_phase_difference=0,
#                     direction=-1
#                 )
#                 group2 = MotorGroup(
#                     motors=[self.motors['M1'], self.motors['M4']],
#                     group_phase_difference=180,
#                     direction=-1
#                 )
#             elif direction == 'right':
#                 group1 = MotorGroup(
#                     motors=[self.motors['M1'], self.motors['M3']],
#                     group_phase_difference=0,
#                     direction=-1
#                 )
#                 group2 = MotorGroup(
#                     motors=[self.motors['M2'], self.motors['M4']],
#                     group_phase_difference=180,
#                     direction=1
#                 )
#             elif direction == 'left':
#                 group1 = MotorGroup(
#                     motors=[self.motors['M1'], self.motors['M3']],
#                     group_phase_difference=0,
#                     direction=1
#                 )
#                 group2 = MotorGroup(
#                     motors=[self.motors['M2'], self.motors['M4']],
#                     group_phase_difference=180,
#                     direction=-1
#                 )
#             elif direction == 'stable':
#                 # No movement
#                 group1 = MotorGroup(motors=[], group_phase_difference=0, direction=1)
#                 group2 = MotorGroup(motors=[], group_phase_difference=0, direction=1)
#             else:
#                 raise ValueError("Invalid direction")
#             logging.debug(f"Configured motor groups in 'normal' mode for direction '{direction}'.")
#         return [group1, group2]

#     def control_loop(self):
#         while self.running:
#             with self.lock:
#                 direction = self.current_direction
#                 mode = self.mode

#             if direction != 'stable':
#                 motor_groups = self.configure_motor_groups(direction, mode)
#                 group1, group2 = motor_groups

#                 base_position = self.generate_sawtooth_position()

#                 group1_targets = group1.generate_target_positions(base_position)
#                 group2_targets = group2.generate_target_positions(base_position)

#                 for motor, target in zip(group1.motors, group1_targets):
#                     motor.move_to_position(target, self.mcp)

#                 for motor, target in zip(group2.motors, group2_targets):
#                     motor.move_to_position(target, self.mcp)
#             else:
#                 # Stop all motors
#                 for motor in self.motors.values():
#                     motor.stop_motor()
#                 logging.debug("All motors stopped (direction: stable).")

#             time.sleep(0.02)  # 20 ms delay

#     def stop(self):
#         self.running = False
#         self.control_thread.join()
#         for motor in self.motors.values():
#             motor.stop_motor()

#         self.motor1_pwm.stop()
#         self.motor2_pwm.stop()
#         self.motor3_pwm.stop()
#         self.motor4_pwm.stop()
#         GPIO.cleanup()
#         logging.info("MotorControlSystem stopped and GPIO cleaned up.")


# motor_control.py
# motor_control.py

import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import threading
import logging

# Configure Logging
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s [%(levelname)s] %(message)s',
                    handlers=[
                        logging.FileHandler("motor_control.log"),
                        logging.StreamHandler()
                    ])

# Constants for SPI and ADC
SPI_PORT = 0
SPI_DEVICE = 0
ADC_MAX = 1023
MIN_ANGLE = 0
MAX_ANGLE = 330
SAWTOOTH_PERIOD = 2  # Period in seconds

# GPIO Pins for Motor 1 (BCM numbering)
MOTOR1_IN1 = 4    # was 7
MOTOR1_IN2 = 7    # was 26
MOTOR1_SPD = 24   # was 18
MOTOR1_ADC_CHANNEL = 0

# GPIO Pins for Motor 2 (BCM numbering)
MOTOR2_IN1 = 5    # was 29
MOTOR2_IN2 = 25   # was 22
MOTOR2_SPD = 6    # was 31
MOTOR2_ADC_CHANNEL = 1

# GPIO Pins for Motor 3 (BCM numbering)
MOTOR3_IN1 = 17   # was 11
MOTOR3_IN2 = 12   # was 32
MOTOR3_SPD = 13   # was 33
MOTOR3_ADC_CHANNEL = 2

# GPIO Pins for Motor 4 (BCM numbering)
MOTOR4_IN1 = 18   # was 12
MOTOR4_IN2 = 27   # was 13
MOTOR4_SPD = 19   # was 35
MOTOR4_ADC_CHANNEL = 3

class SpikeFilter:
    def __init__(self, name):
        self.filter_active = False
        self.last_valid_reading = None
        self.name = name
        # Adding constants based on requirements
        self.DEAD_ZONE_THRESHOLD = 950  # Activation threshold before dead zone
        self.LOWER_VALID_LIMIT = 150    # ~45 degrees
        self.UPPER_VALID_LIMIT = 750    # Balanced threshold for backward movement
        self.ADC_MAX = 1023             # Maximum ADC reading

    def filter(self, new_value):
        # Case 1: Check for entering dead zone
        if not self.filter_active and self.last_valid_reading is not None:
            if self.last_valid_reading >= self.DEAD_ZONE_THRESHOLD:
                self.filter_active = True
                logging.debug(f"{self.name} SpikeFilter: Entering dead zone. Last valid: {self.last_valid_reading}")
                return self.last_valid_reading

        # Case 2: Filter is active (in dead zone)
        if self.filter_active:
            # Check if we've exited the dead zone with a valid reading
            if self.LOWER_VALID_LIMIT <= new_value <= self.UPPER_VALID_LIMIT:
                self.filter_active = False
                self.last_valid_reading = new_value
                logging.debug(f"{self.name} SpikeFilter: Exited dead zone with valid reading {new_value}")
                return new_value
            else:
                # Still in dead zone, return last valid reading
                return self.last_valid_reading if self.last_valid_reading is not None else 0

        # Case 3: Normal operation (filter not active)
        # Check for sudden spikes that might indicate entering dead zone
        if self.last_valid_reading is not None and abs(self.last_valid_reading - new_value) > 300:
            self.filter_active = True
            logging.debug(f"{self.name} SpikeFilter: Sudden spike detected: {new_value}")
            return self.last_valid_reading
        
        # Normal valid reading
        self.last_valid_reading = new_value
        return new_value

    def reset(self):
        self.filter_active = False
        self.last_valid_reading = None
        logging.debug(f"{self.name} SpikeFilter: Reset complete.")

class PIDController:
    def __init__(self, Kp, Ki, Kd, name='PID'):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()
        self.name = name

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

        logging.debug(f"{self.name} PID: error={error}, proportional={proportional}, integral={integral}, derivative={derivative}, control_signal={control_signal}")

        return control_signal

    def reset(self):
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()
        logging.debug(f"{self.name} PID: Reset complete.")

class MotorController:
    def __init__(self, name, in1, in2, pwm, adc_channel, encoder_flipped=False):
        self.name = name
        self.in1 = in1
        self.in2 = in2
        self.pwm = pwm
        self.adc_channel = adc_channel
        self.position = 0
        self.last_valid_position = None
        self.pid = PIDController(Kp=0.8, Ki=0.1, Kd=0.05, name=f'PID-{self.name}')
        self.encoder_flipped = encoder_flipped
        self.spike_filter = SpikeFilter(name)

    def read_position(self, mcp):
        raw_value = mcp.read_adc(self.adc_channel)
        
        if self.encoder_flipped:
            raw_value = ADC_MAX - raw_value
        
        filtered_value = self.spike_filter.filter(raw_value)
        
        if filtered_value is None:
            return self.last_valid_position if self.last_valid_position is not None else 0
            
        degrees = (filtered_value / ADC_MAX) * 330.0
        self.last_valid_position = degrees
        logging.debug(f"{self.name} Position: {degrees}° (raw: {raw_value})")
        return degrees

    def set_motor_direction(self, direction):
        if direction == 'forward':
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
            logging.debug(f"{self.name} Direction: Forward")
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
            logging.debug(f"{self.name} Direction: Backward")

    def move_to_position(self, target, mcp):
        try:
            current_position = self.read_position(mcp)
            error = target - current_position

            if error > 165:
                error -= 330
            elif error < -165:
                error += 330

            control_signal = self.pid.compute(error)

            if abs(error) <= 2:
                self.stop_motor()
                logging.debug(f"{self.name} reached target. Stopping motor.")
                return True

            direction = 'forward' if control_signal > 0 else 'backward'
            self.set_motor_direction(direction)
            speed = min(100, max(30, abs(control_signal)))
            self.pwm.ChangeDutyCycle(speed)
            logging.debug(f"{self.name} Speed: {speed}%")
            return False
        except Exception as e:
            logging.error(f"{self.name} MotorController: Error during move_to_position: {e}")
            self.reset()
            return False

    def stop_motor(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
        logging.debug(f"{self.name} Motor Stopped")

    def reset(self):
        self.stop_motor()
        self.pid.reset()
        self.spike_filter.reset()
        logging.info(f"{self.name} MotorController: Reset complete.")

class MotorGroup:
    def __init__(self, motors, group_phase_difference=0, direction=1):
        self.motors = motors
        self.group_phase_difference = group_phase_difference
        self.direction = direction

    def generate_target_positions(self, base_position):
        target_positions = []
        for motor in self.motors:
            position = (base_position + self.group_phase_difference) % MAX_ANGLE
            if self.direction == -1:
                position = (MAX_ANGLE - position) % MAX_ANGLE
            target_positions.append(position)
        return target_positions

class MotorControlSystem:
    def __init__(self, mode='normal'):
        self.mode = mode  # 'normal' or 'gallop'
        self.current_direction = 'stable'
        self.lock = threading.Lock()
        self.running = True

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(MOTOR1_IN1, GPIO.OUT)
        GPIO.setup(MOTOR1_IN2, GPIO.OUT)
        GPIO.setup(MOTOR1_SPD, GPIO.OUT)
        GPIO.setup(MOTOR2_IN1, GPIO.OUT)
        GPIO.setup(MOTOR2_IN2, GPIO.OUT)
        GPIO.setup(MOTOR2_SPD, GPIO.OUT)
        GPIO.setup(MOTOR3_IN1, GPIO.OUT)
        GPIO.setup(MOTOR3_IN2, GPIO.OUT)
        GPIO.setup(MOTOR3_SPD, GPIO.OUT)
        GPIO.setup(MOTOR4_IN1, GPIO.OUT)
        GPIO.setup(MOTOR4_IN2, GPIO.OUT)
        GPIO.setup(MOTOR4_SPD, GPIO.OUT)

        # Set up PWM for motor speed control
        self.motor1_pwm = GPIO.PWM(MOTOR1_SPD, 1000)
        self.motor1_pwm.start(0)
        self.motor2_pwm = GPIO.PWM(MOTOR2_SPD, 1000)
        self.motor2_pwm.start(0)
        self.motor3_pwm = GPIO.PWM(MOTOR3_SPD, 1000)
        self.motor3_pwm.start(0)
        self.motor4_pwm = GPIO.PWM(MOTOR4_SPD, 1000)
        self.motor4_pwm.start(0)

        # Set up MCP3008
        self.mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

        # Initialize motors
        self.motor1 = MotorController("M1", MOTOR1_IN1, MOTOR1_IN2, self.motor1_pwm,
                                      MOTOR1_ADC_CHANNEL, encoder_flipped=False)
        self.motor2 = MotorController("M2", MOTOR2_IN1, MOTOR2_IN2, self.motor2_pwm,
                                      MOTOR2_ADC_CHANNEL, encoder_flipped=True)
        self.motor3 = MotorController("M3", MOTOR3_IN1, MOTOR3_IN2, self.motor3_pwm,
                                      MOTOR3_ADC_CHANNEL, encoder_flipped=False)
        self.motor4 = MotorController("M4", MOTOR4_IN1, MOTOR4_IN2, self.motor4_pwm,
                                      MOTOR4_ADC_CHANNEL, encoder_flipped=True)

        self.motors = {
            'M1': self.motor1,
            'M2': self.motor2,
            'M3': self.motor3,
            'M4': self.motor4
        }

        self.start_time = time.time()

        # Start the control loop in a separate thread
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()

        logging.info(f"MotorControlSystem initialized in '{self.mode}' mode.")

    def set_direction(self, direction):
        with self.lock:
            self.current_direction = direction
            logging.info(f"Direction set to '{direction}'.")
            if direction == 'stable':
                logging.info("Resetting all motors to 'stable' state.")
                for motor in self.motors.values():
                    motor.reset()

    def set_mode(self, mode):
        with self.lock:
            self.mode = mode
            logging.info(f"Mode set to '{mode}'.")

    def get_status(self):
        with self.lock:
            status = {
                'mode': self.mode,
                'direction': self.current_direction,
                'motors': {}
            }

            for name, motor in self.motors.items():
                pos = motor.read_position(self.mcp)
                status['motors'][name] = pos

            # Calculate phase differences
            m1_pos = status['motors']['M1']
            m2_pos = status['motors']['M2']
            m3_pos = status['motors']['M3']
            m4_pos = status['motors']['M4']

            phase_diff_m1_m3 = abs(m1_pos - m3_pos)
            phase_diff_m2_m4 = abs(m2_pos - m4_pos)

            phase_diff_m1_m3 = min(phase_diff_m1_m3, MAX_ANGLE - phase_diff_m1_m3)
            phase_diff_m2_m4 = min(phase_diff_m2_m4, MAX_ANGLE - phase_diff_m2_m4)

            status['phase_diff'] = {
                'M1-M3': phase_diff_m1_m3,
                'M2-M4': phase_diff_m2_m4
            }

            return status

    def generate_sawtooth_position(self):
        elapsed_time = time.time() - self.start_time
        position_in_cycle = (elapsed_time % SAWTOOTH_PERIOD) / SAWTOOTH_PERIOD
        position = (position_in_cycle * MAX_ANGLE) % MAX_ANGLE
        return position

    def configure_motor_groups(self, direction, mode):
        if mode == 'gallop' and direction in ['forward', 'backward']:
            # In gallop mode, M4 and M3 are synced with 180-degree phase difference to M2 and M1
            group1 = MotorGroup(
                motors=[self.motors['M2'], self.motors['M1']],
                group_phase_difference=0,
                direction=1 if direction == 'forward' else -1
            )
            group2 = MotorGroup(
                motors=[self.motors['M4'], self.motors['M3']],
                group_phase_difference=180,
                direction=1 if direction == 'forward' else -1
            )
            logging.debug(f"Configured motor groups in 'gallop' mode for direction '{direction}'.")
        else:
            # Normal mode or turning commands remain unchanged
            if direction == 'forward':
                group1 = MotorGroup(
                    motors=[self.motors['M2'], self.motors['M3']],
                    group_phase_difference=0,
                    direction=1
                )
                group2 = MotorGroup(
                    motors=[self.motors['M1'], self.motors['M4']],
                    group_phase_difference=180,
                    direction=1
                )
            elif direction == 'backward':
                group1 = MotorGroup(
                    motors=[self.motors['M2'], self.motors['M3']],
                    group_phase_difference=0,
                    direction=-1
                )
                group2 = MotorGroup(
                    motors=[self.motors['M1'], self.motors['M4']],
                    group_phase_difference=180,
                    direction=-1
                )
            elif direction == 'right':
                group1 = MotorGroup(
                    motors=[self.motors['M1'], self.motors['M3']],
                    group_phase_difference=0,
                    direction=-1
                )
                group2 = MotorGroup(
                    motors=[self.motors['M2'], self.motors['M4']],
                    group_phase_difference=180,
                    direction=1
                )
            elif direction == 'left':
                group1 = MotorGroup(
                    motors=[self.motors['M1'], self.motors['M3']],
                    group_phase_difference=0,
                    direction=1
                )
                group2 = MotorGroup(
                    motors=[self.motors['M2'], self.motors['M4']],
                    group_phase_difference=180,
                    direction=-1
                )
            elif direction == 'stable':
                # No movement
                group1 = MotorGroup(motors=[], group_phase_difference=0, direction=1)
                group2 = MotorGroup(motors=[], group_phase_difference=0, direction=1)
            else:
                raise ValueError("Invalid direction")
            logging.debug(f"Configured motor groups in 'normal' mode for direction '{direction}'.")
        return [group1, group2]

    def control_loop(self):
        while self.running:
            with self.lock:
                direction = self.current_direction
                mode = self.mode

            if direction != 'stable':
                motor_groups = self.configure_motor_groups(direction, mode)
                group1, group2 = motor_groups

                base_position = self.generate_sawtooth_position()

                group1_targets = group1.generate_target_positions(base_position)
                group2_targets = group2.generate_target_positions(base_position)

                for motor, target in zip(group1.motors, group1_targets):
                    motor.move_to_position(target, self.mcp)

                for motor, target in zip(group2.motors, group2_targets):
                    motor.move_to_position(target, self.mcp)
            else:
                # Stop all motors
                for motor in self.motors.values():
                    motor.stop_motor()
                logging.debug("All motors stopped (direction: stable).")

            time.sleep(0.02)  # 20 ms delay

    def stop(self):
        self.running = False
        self.control_thread.join()
        for motor in self.motors.values():
            motor.stop_motor()

        self.motor1_pwm.stop()
        self.motor2_pwm.stop()
        self.motor3_pwm.stop()
        self.motor4_pwm.stop()
        GPIO.cleanup()
        logging.info("MotorControlSystem stopped and GPIO cleaned up.")
