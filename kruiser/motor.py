
'''
   3V3  (1) (2)  5V    
 GPIO2  (3) (4)  5V    
 GPIO3  (5) (6)  GND   
 GPIO4  (7) (8)  GPIO14
   GND  (9) (10) GPIO15
GPIO17 (11) (12) GPIO18
GPIO27 (13) (14) GND   
GPIO22 (15) (16) GPIO23
   3V3 (17) (18) GPIO24
GPIO10 (19) (20) GND   
 GPIO9 (21) (22) GPIO25
GPIO11 (23) (24) GPIO8 
   GND (25) (26) GPIO7 
 GPIO0 (27) (28) GPIO1 
 GPIO5 (29) (30) GND   
 GPIO6 (31) (32) GPIO12
GPIO13 (33) (34) GND   
GPIO19 (35) (36) GPIO16
GPIO26 (37) (38) GPIO20
   GND (39) (40) GPIO21
'''

import pigpio
import time

class Drivetrain:
    def __init__(self, rear1, rear2, rearPWM, front1, front2, frontPWM):
        self.motorR = Motor(rear1,rear2, rearPWM)
        self.motorF = Motor(front1,front2, frontPWM)
        
    def brake(self):
        self.motorR.brake()
        self.motorF.brake()
        
    def stop(self):
        self.motorR.stop()
        self.motorF.stop()
        
    def delayMillis(self, duration):
        if duration > 0:
            time.sleep(duration/1000)
            
    def turn(self, pct, duration = 0):
        self.motorF.setSpeed(pct/100)
        self.delayMillis(duration)
        
    def forward(self, pct, duration = 0):
        self.motorR.setSpeed(pct/100)
        self.delayMillis(duration)
        
    def reverse(self, pct, duration = 0):
        self.motorR.setSpeed(-pct/100)
        self.delayMillis(duration)
        
    def drive(self, speed, turnpct, duration = 0):
        self.turn(turnpct)
        self.forward(speed)
        self.delayMillis(duration)
        
    
        
class Motor:
    def __init__(self, pin1, pin2, pwm):
        self.pin1 = pin1
        self.pin2 = pin2
        self.pwm = pwm
        self.pi = pigpio.pi()
        self.speed = 0

        for i in [pin1,pin2,pwm]:
            self.pi.set_mode(i,pigpio.OUTPUT)
            
        self.pi.set_PWM_frequency(pwm, 50)
        self.brake()
        
    ''' Brake '''
    def brake(self): 
        for pin in [self.pin1,self.pin2]:
            self.pi.write(pin,0)
        self.stop()
        
    ''' Stop '''
    def stop(self):
        self.pi.hardware_PWM(self.pwm, 50, 0)
        
    ''' Turn pin on '''
    def on(self, pin):
        self.pi.write(pin, 1)
        
    ''' Turn pin off '''
    def off(self,pin):
        self.pi.write(pin, 0)
        
    ''' Set Speed '''
    def setSpeed(self, speed):
        if speed < 0:
            self.on(self.pin1)
            self.off(self.pin2)
        else:
            self.off(self.pin1)
            self.on(self.pin2)
        
        self.speed = speed
        self.pi.hardware_PWM(self.pwm, 50, int(abs(speed)*1000000))
        

    
        
        