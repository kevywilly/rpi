import pigpio

'''
import pigpio

pi = pigpio.pi()
freq = 50

pi.set_mode(5, pigpio.OUTPUT)
pi.set_mode(6, pigpio.OUTPUT)
pi.set_mode(13, pigpio.OUTPUT)

pi.write(5,0)
pi.write(6,1)

pi.hardware_PWM(13,50,500000)

'''
    
class Servo:
    def __init__(self, pin, frequency = 50, min_pulse_width = 500, max_pulse_width = 2400):
        self.pin = pin
        self.min_pulse_width = min_pulse_width
        self.max_pulse_width = max_pulse_width
        self.frequency = frequency
        self.pi = pigpio.pi()
        self.angle = 0
        self.pulse_width = 0

    def start(self):
        self.pi.set_mode(self.pin, pigpio.OUTPUT)
        self.pi.set_PWM_frequency(self.pin, self.frequency)
        self.disable()
        
    def disable(self):
        self.pi.set_servo_pulsewidth(self.pin, 0)
        
        
    def setAngle(self, angle):
        if angle < -90:
            angle = -90
        elif angle > 90:
            angle = 90
        
        self.pulse_width = self.map(angle, -90, 90, self.min_pulse_width, self.max_pulse_width)
        self.pi.set_servo_pulsewidth(self.pin, self.pulse_width)
        
    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
