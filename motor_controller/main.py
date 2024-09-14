import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
from motor_pins import MOTOR_PINS
from motor_controller import MotorController
from pid_controller import PIDController
from filter import Filter

SPI_PORT = 0
SPI_DEVICE = 0
OFFSET = 5
DEAD_ZONE_DEG_START = 330
DEAD_ZONE_DEG_END = 360
Kp = 0.5  # Increased proportional gain
Ki = 0.05  # Increased integral gain
Kd = 0.1  # Increased derivative gain
MAX_FILTER_COUNT = 5
NUM_SAMPLES_FOR_AVERAGE = 5

def millis():
    return int(time.time() * 1000)

def map_potentiometer_value_to_degrees(value):
    return value * (360 / 1023)

def sawtooth_wave(t, period, amplitude, initial_angle):
    normalized_wave = (t % period) * (amplitude / period)
    set_position = (normalized_wave + initial_angle) % 360
    return set_position

def main():
    mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))
    motor_pins = MOTOR_PINS[4]  # Example for motor 4
    motor = MotorController(motor_pins['IN1'], motor_pins['IN2'], motor_pins['SPD'])
    pid = PIDController(Kp, Ki, Kd)
    filter = Filter(MAX_FILTER_COUNT)

    try:
        initial_pot_value = mcp.read_adc(3)
        initial_angle = map_potentiometer_value_to_degrees(initial_pot_value)
        start_time = millis()

        # Move motor to 180 degrees at the start
        target_angle = 180
        while True:
            pot_value = mcp.read_adc(3)
            filtered_pot_value = filter.apply(pot_value)
            if filtered_pot_value is None:
                continue
            degrees_value = map_potentiometer_value_to_degrees(filtered_pot_value)
            control_signal = pid.calculate(target_angle, degrees_value)
            motor.set_speed(control_signal)

            # Print the values
            print(f"Potentiometer Value: {pot_value}, Degrees: {degrees_value:.2f}, Target Angle: {target_angle}, Control Signal: {control_signal:.2f}%")

            # Check if the motor has reached the target angle within the offset range
            if abs(degrees_value - target_angle) <= OFFSET:
                motor.stop()
                break

            time.sleep(0.1)

        # Wait for 5 seconds
        print("Motor reached 180 degrees. Waiting for 5 seconds...")
        time.sleep(5)

        # Main control loop
        while True:
            pot_value = mcp.read_adc(3)
            filtered_pot_value = filter.apply(pot_value)
            if filtered_pot_value is None:
                continue
            degrees_value = map_potentiometer_value_to_degrees(filtered_pot_value)
            current_time = millis()
            set_position = sawtooth_wave(current_time - start_time, 10000, 360, initial_angle)  # Increased period to 10 seconds
            control_signal = pid.calculate(set_position, degrees_value)
            motor.set_speed(control_signal)

            # Print the values
            print(f"Potentiometer Value: {pot_value}, Degrees: {degrees_value:.2f}, Set Position: {set_position:.2f}, Control Signal: {control_signal:.2f}%")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Program stopped by user")
    finally:
        motor.cleanup()

if __name__ == "__main__":
    main()