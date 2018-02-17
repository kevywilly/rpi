
import time

# Import SPI library (for hardware SPI) and MCP3008 library.
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

SPI_PORT   = 0
SPI_DEVICE = 0
        
class SensorArray:
    def __init__(self):
        self.ir_right = 2
        self.ir_left = 1
        self.ir_front = 0
        self.values = [0]*8
        
        self.mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))
        
    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        

    def ir_to_cm(self, value):
        return self.map(value,7,880,30,4)
            
    def getDistances(self):
        result = [30]*3
        for i in [self.ir_front, self.ir_left, self.ir_right]:
            result[i] = int(self.ir_to_cm(self.mcp.read_adc(i)))
            
        return result
        