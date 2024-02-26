import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# create the spi bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D5)

# create the mcp object
mcp = MCP.MCP3008(spi, cs)

# create an analog input channel on pin 0
chan = AnalogIn(mcp, MCP.P0)

def adc_value_to_angle(adc_value):
    # Assuming adc_value is scaled to 16-bit (0 to 65535)
    # Map this value to 0 to 360 degrees
    return (adc_value / 65535) * 360

while True:
    # Read the scaled 16-bit ADC value
    raw_adc_value = chan.value
    # Convert the raw ADC value to an angle
    angle = adc_value_to_angle(raw_adc_value)
    
    print("Raw ADC Value: ", raw_adc_value)
    print("Angle: {:.2f} degrees".format(angle))
