
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
from server import *
import pigpio

FORWARD = 1
REVERSE = 2
RIGHT = 3
LEFT = 4

# servos
servo1 = Servo(17, min_pulse_width = 510, max_pulse_width = 2390)
servo2 = Servo(27, min_pulse_width = 510, max_pulse_width = 2390)

motors = Drivetrain(15,14,18,6,5,13)

# ir sensors
sensors = SensorArray()
    
HOST, PORT = "localhost", 9000
server = ThreadedTCPServer((HOST, PORT), ThreadedTCPRequestHandler)

def cleanup():
    motors.brake()
    

def client(ip, port, message):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ip, port))
    try:
        sock.sendall(message)
        response = sock.recv(1024)
        print ("Received: {}".format(response))
    finally:
        sock.close()

def sig_handler(signum, frame):
    global server
    cleanup()
    print('Signal handler called with signal', signum)
    server.shutdown
    sys.exit(0)
    
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
    
    
signal.signal(signal.SIGINT, sig_handler)
signal.signal(signal.SIGTERM, sig_handler)

if __name__ == "__main__":
    # Port 0 means to select an arbitrary unused port
    #HOST, PORT = "localhost", 0

    #server = ThreadedTCPServer((HOST, PORT), ThreadedTCPRequestHandler)
   

    ip, port = server.server_address

    # Start a thread with the server -- that thread will then start one
    # more thread for each request
    server_thread = threading.Thread(target=server.serve_forever)
    # Exit the server thread when the main thread terminates
    server_thread.daemon = True
    server_thread.start()
    print ("Server loop running in thread:", server_thread.name)

    client(ip, port, b"Hello World 1")
    client(ip, port, b"Hello World 2")
    client(ip, port, b"Hello World 3")
    
    while True:
        print("running")
        time.sleep(1)

    server.shutdown()
    server.server_close()
    
    
    cleanup()
    
    #raspistill -o output.jpg
    


            
    
    
    