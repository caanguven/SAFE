import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
from time import sleep
def setup_adc():
    # Create the SPI bus
    spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

    # Create the CS (chip select)
    cs = digitalio.DigitalInOut(board.D5)

    # Create the MCP object
    mcp = MCP.MCP3008(spi, cs)
    return mcp

def adc_value_to_angle(adc_value):
    #map 
    return ((adc_value / 1023) * 360)

def read_position():
    mcp = setup_adc()
    # Create an analog input channel on pin 0
    chan = AnalogIn(mcp, MCP.P0)
    raw_adc_value = chan.value >> 6
    print('voltage', chan.voltage)
    # print('raw ADC', raw_adc_value)
    angle = adc_value_to_angle(raw_adc_value)
    return angle

while True:
    current_position = read_position()
    # print('test',current_position)

    sleep(0.1)


