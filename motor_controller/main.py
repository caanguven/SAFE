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
Kp = 0.1
Ki = 0.01
Kd = 0.02
MAX_FILTER_COUNT = 5
NUM_SAMPLES_FOR_AVERAGE = 5

def millis():
    return int(time.time() * 1000)

def map_potentiometer_value_to_degrees(value):
    return value * (360 / 1023)

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

        while True:
            pot_value = mcp.read_adc(3)
            filtered_pot_value = filter.apply(pot_value)
            if filtered_pot_value is None:
                continue
            degrees_value = map_potentiometer_value_to_degrees(filtered_pot_value)
            current_time = millis()
            set_position = (current_time - start_time) % 360
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