import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
from motor_pins import setup_motor_pins, control_motor
from pid_controller import pid_control
from utils import map_potentiometer_value_to_degrees, custom_spike_filter
import RPi.GPIO as GPIO  # Make sure GPIO is properly imported

# MCP3008 setup
SPI_PORT = 0
SPI_DEVICE = 0
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

# Setup for a specific motor
MOTOR_NUM = 4
setup_motor_pins(MOTOR_NUM)

# Motor control loop
def adc_and_motor_control():
    pwm = None  # Initialize pwm variable to None
    try:
        last_valid_reading = None
        filter_active = False
        filter_count = 0
        start_time = time.time()

        while True:
            # Read potentiometer value from channel 3 (for motor 4)
            pot_value = mcp.read_adc(3)
            filtered_pot_value, filter_active, filter_count = custom_spike_filter(
                pot_value, last_valid_reading, filter_active, filter_count)

            if filtered_pot_value is None:
                continue

            degrees_value = map_potentiometer_value_to_degrees(filtered_pot_value)

            # Set position dynamically (e.g., using a wave or constant value)
            set_position = 180  # Example set position

            # Use PID to calculate the control signal
            control_signal = pid_control(set_position, degrees_value)

            # Apply the control signal to the motor
            pwm = control_motor(MOTOR_NUM, "forward", control_signal)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping motor control")
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        if pwm:
            pwm.stop()  # Stop PWM only if it was started
        GPIO.cleanup()  # Make sure GPIO.cleanup() is always called

if __name__ == "__main__":
    adc_and_motor_control()
