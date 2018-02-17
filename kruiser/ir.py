
import pigpio

class IRReader:
    
    def __init__(self, addr = 0x04, bus = 1, sensors = 3): 
        self.pi = pigpio.pi()
        self.handle = self.pi.i2c_open(bus,addr)
        self.sensors = sensors
        
        
    def readSensors(self):
        (count,bytes) = self.pi.i2c_read_device(self.handle, self.sensors)
        return bytes
