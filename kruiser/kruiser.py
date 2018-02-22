
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

import socket
import threading
import socketserver
import signal, os, sys
import time
from servo import Servo
from sensor import SensorArray
from motor import Motor, Drivetrain
import json
import pigpio
import random

FORWARD = 1
REVERSE = 2
RIGHT = 3
HARDRIGHT = 4
LEFT = 5
HARDLEFT = 6
SPEED = 75
SPEED2 = 50

rand = -1

# servos
servo1 = Servo(17, min_pulse_width = 510, max_pulse_width = 2390)
servo2 = Servo(27, min_pulse_width = 510, max_pulse_width = 2390)

motors = Drivetrain(15,14,18,6,5,13)

# ir sensors
sensors = SensorArray()
    
HOST, PORT = "0.0.0.0", 9001

server = None
    
class Command:
    
    def __init__(self, json_string):
        self.json_data = json.loads('{}')
        try: 
            self.json_data = json.loads(json_string)
            self.cmd = self.json_data["cmd"]
        except:
            print("Could not read JSON")
            self.cmd = None
        
        
    def get(self,key):
        if key in self.json_data:
            return self.json_data[key]
        return None
        
        
    def get_or(self,key,default):
        v = self.get(key)
        if v == None:
            return default
        
        return v
        
class EchoRequestHandler(socketserver.BaseRequestHandler):

    def handle(self):
        # Echo the back to the client
        data = self.request.recv(256)
        dispatch(data)
        self.request.send(b'ok')
        return
       
def close_server():
	global server
	try:
		server.shutdown(1)
	except:
		print("server not connected")
	finally:
		server.socket.close()

def signal_handler(signal, frame):
    
    print('You pressed Ctrl+C!')

    try:
        cleanup()
        close_server()
    except:
        pass
    finally:
        sys.exit(0)


def start_server(port):
    global server
    
    server = socketserver.TCPServer(('0.0.0.0',port), EchoRequestHandler)
    signal.signal(signal.SIGINT, signal_handler)
    t = threading.Thread(target=server.serve_forever)
    t.setDaemon(True) # don't hang on exit
    t.start()
    print("started server ... waiting for connections")
    
    while 1:
        pass
    
# Dispatch message
def dispatch(data):
    
    s = str(data[0:(data.find(0))],"utf-8")
    print(s)
    
    command = Command(s)
    cmd = command.cmd
    
    print(cmd)
    if cmd == "drive":
        #motors.brake()
        motors.drive(int(command.get_or("speed",0)*100), int(command.get_or("turn",0)*100))
    elif cmd == "look":
        servo1.setAngle(int(command.get_or("yaw",0)))
        
        p = int(command.get_or("pitch",0)) + 90
        if p > 90:
            p = 90
        servo2.setAngle(p)
        
    

def cleanup():
    try: 
        motors.brake()
    except:
        "stopped motors"

def autonomous(setmode=FORWARD):
    global rand
    sensors.read_adc()
    score = sensors.score
    right, left = score
    mode = setmode
    
    if mode == RIGHT and left <= 0b001:
        mode = FORWARD
    elif mode == HARDRIGHT and left < 0b010:
        mode = FORWARD
    elif mode == LEFT and right <= 0b001:
        mode = FORWARD
    elif mode == HARDLEFT and right < 0b010:
        mode = FORWARD
    elif mode == REVERSE and score == (0,0):
        mode = FORWARD
    elif score == (0,0):
        mode = FORWARD
    elif score == (0b111,0b111):
        mode = REVERSE
    elif right > 0b011 and left > 0b011:
        mode = REVERSE
    elif right > 0 and left > 0:
        if left > right:
            mode = RIGHT
        else:
            mode = LEFT
    elif right > left:
        mode = LEFT
        if right > 0b011:
            mode = HARDLEFT
    elif right < left:
        mode = RIGHT
        if left > 0b011:
            mode = HARDRIGHT
    else:
        mode = FORWARD
    
    if mode is not REVERSE:
        rand = -1
        
    # NOW RUN THE MOTORS
    if mode == FORWARD:
        motors.drive(SPEED,0)
    elif mode == REVERSE:
        if rand == -1:
            rand = random.randint(1,2)
        if rand == 1:
            motors.drive(-SPEED2, 100, 200)
        else:
            motors.drive(-SPEED2, -100, 200)
    elif mode == RIGHT:
        motors.drive(SPEED2, 60,25)
    elif mode == LEFT:
        motors.drive(SPEED2, -60,25)
    elif mode == HARDRIGHT:
        motors.drive(SPEED2, 100,50)
    elif mode == HARDLEFT:
        motors.drive(SPEED2, -100,50)
        
    return mode
    
'''    
def autonomous():
    
    global run_mode
    
    
 
    mode = FORWARD
    while(True):
        try:
            f,l,r = sensors.getDistances()
            
            print(f, " ", l, " ", r)
            
            if mode is not REVERSE and f < 15:
                mode = FORWARD
                motors.stop()
                
            if mode is FORWARD:
                if f < 15: # close obstacle front
                    mode = REVERSE
                    motors.stop()
                    if l < r:
                        motors.drive(-40, 90, duration = 1000)
                    else:
                        motors.drive(-40, -90, duration = 1000)
                elif f < 29:   # obstacle front
                    if l < r:
                        motors.drive(40,90)
                        mode = RIGHT
                    else:
                        motors.drive(40,-90)
                        mode = LEFT
                elif l < 20 and l < r: #obstacle left
                    motors.drive(30, 40)
                    mode = RIGHT
                elif r < 20 and r < l: #obstacle right
                    motors.drive(30,-40)
                    mode = LEFT
                else:
                    mode = FORWARD
                    motors.drive(50,0)
            elif mode is REVERSE:
                if f >= 28:
                    mode = FORWARD
            elif mode is RIGHT:
                if f >= 28 and l > 20:
                    mode = FORWARD
            elif mode is LEFT:
                if f >=28 and r > 20:
                    mode = FORWARD
                    
            time.sleep(15/1000)
            
        except Exception as e:
            print(e)
    
'''

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

if __name__ == "__main__":
   
    random.seed(time.time)
    servo1.setAngle(0) # yaw
    servo2.setAngle(80) # pitch
    
    ''' run server '''
    #start_server(8000)
    
    ''' or autonomous '''
    mode = FORWARD
    while True:
        print(sensors.score)
        mode = autonomous(mode)
        time.sleep(15/1000)
    
    close_server()
    cleanup()
    
    #raspistill -o output.jpg
    


            
    
    
    