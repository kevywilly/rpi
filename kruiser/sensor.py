
import time

IR_FR1 = 0
IR_FR2 = 1
IR_FL1 = 2
IR_FL2 = 3
IRR0 = 0
IRR1 = 1
IRRC = 2
# Import SPI library (for hardware SPI) and MCP3008 library.
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

SPI_PORT   = 0
SPI_DEVICE = 0
THRESHOLDS = [25,20,15,25,20,15]
class SensorArray:
    def __init__(self):
        self.score = (0,0)
        self.values = [0]*6
        self.distances = [30]*6
        
        self.mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))
        
    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        
    def read_adc(self):
        for i in range(0,6):
            d = self.mcp.read_adc(i)
            self.values[i] = d
            self.distances[i] = int(self.ir_to_cm(d))
        self.calc_score()

    def calc_score(self):
        s0 = 0
        s1 = 0
        j = 2
        for i in range(0,3):
            if self.distances[i] <= THRESHOLDS[i]:
                s0 |= (1 << j)
            if self.distances[i+3] <= THRESHOLDS[i+3]:
                s1 |= (1 << j)
            j -= 1
            
        self.score = (s0,s1)
        
    def ir_to_cm(self, value):
        return self.map(value,7,880,30,4)
        
    def reccomend(self):
        self.read_adc()
        
'''
from sensor import SensorArray
from time import sleep
s = SensorArray()
while True:
    s.read_adc()
    print(s.distances)
    print(s.score)
    sleep(100/1000)
'''