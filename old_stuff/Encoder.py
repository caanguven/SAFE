
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import RPi.GPIO as GPIO
import time
import getch

def setup_adc():
    spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
    cs = digitalio.DigitalInOut(board.D5)
    cs.direction = digitalio.Direction.OUTPUT
    mcp = MCP.MCP3008(spi, cs)
    return mcp

mcp = setup_adc()  # Initialize the ADC once, outside of the loop

def adc_value_to_angle(adc_value):
    return ((adc_value / 65535) * 360)

    
def setup_motors_blinka():
    motor_pins = {
        'motor1': {'forward': digitalio.DigitalInOut(board.D7), 'backward': digitalio.DigitalInOut(board.D26)},
        #'motor2': {'forward': digitalio.DigitalInOut(board.D22), 'backward': digitalio.DigitalInOut(board.D29)},
        #'motor3': {'forward': digitalio.DigitalInOut(board.D32), 'backward': digitalio.DigitalInOut(board.D11)},
        #'motor4': {'forward': digitalio.DigitalInOut(board.D13), 'backward': digitalio.DigitalInOut(board.D12)},
    }
    for motor in motor_pins.values():
        for pin in motor.values():
            pin.direction = digitalio.Direction.OUTPUT
            pin.value = False
    return motor_pins

# Attempt to setup motors, catch ValueError if mode has been set
try:
    motor_pins = setup_motors_blinka()
    print('motor_setup done')
except ValueError as e:
    print(e)
    GPIO.cleanup()  # Cleanup and try setting up again
    print('trying again')
    motor_pins = setup_motors_blinka()

def motor_control(motor_pins, direction):
    # Set all motor pins to LOW
    for pin in motor_pins.values():
        pin.value = False
    # Set the specific direction to HIGH
    if direction in motor_pins:
        print('turnin')
        print(direction)   
        (board.D7)= True


def read_position(mcp):
    chan = AnalogIn(mcp, MCP.P0)
    raw_adc_value = chan.value >> 6
    angle = adc_value_to_angle(raw_adc_value)
    return angle


try:
    while True:
        motor_control(motor_pins['motor1'], 'backward')
        time.sleep(2)
        current_position = read_position(mcp)
        print('motor1 position:', current_position)
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()  # Ensure a clean exit


