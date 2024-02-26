import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

def setup_adc():
    # Create the SPI bus
    spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

    # Create the CS (chip select)
    cs = digitalio.DigitalInOut(board.D5)

    # Create the MCP object
    mcp = MCP.MCP3008(spi, cs)
    return mcp

def adc_value_to_angle(adc_value):
    # Assuming adc_value is scaled to 16-bit (0 to 65535)
    # Map this value to 0 to 360 degrees
    return (adc_value / 65535) * 360

def read_position():
    mcp = setup_adc()
    # Create an analog input channel on pin 0
    chan = AnalogIn(mcp, MCP.P0)
    raw_adc_value = chan.value
    angle = adc_value_to_angle(raw_adc_value)
    return angle
