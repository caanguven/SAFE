import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import threading

class ADCReader:
    def __init__(self, spi_port=0, spi_device=0):
        self.mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(spi_port, spi_device))
        self.lock = threading.Lock()

    def read_channel(self, channel):
        with self.lock:
            return self.mcp.read_adc(channel)
