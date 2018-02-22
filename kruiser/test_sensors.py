from sensor import SensorArray
from time import sleep
s = SensorArray()
while True:
    s.read_adc()
    print(s.distances)
    print(s.score)
    sleep(100/1000)
