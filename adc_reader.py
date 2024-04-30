import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

# Hardware SPI configuration:
SPI_PORT = 0
SPI_DEVICE = 0
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

def read_adc_values():
    values = [mcp.read_adc(i) for i in range(8)]
    # Convert it to degrees
    degrees = [((value / 1023.0) * 360) % 360 for value in values]
    return degrees


