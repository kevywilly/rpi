from time import sleep
from picamera import PiCamera


def capture_bw:
    camera = picamera.PiCamera()     # initialize camera
    camera.color_effects = (128,128) # turn camera to black and white
    camera.capture('image1.jpg')  
    
def capture_sequence:
    camera = PiCamera(resolution=(640, 480), framerate=30)
    # Set ISO to the desired value
    camera.iso = 400
    # Wait for the automatic gain control to settle
    sleep(2)
    # Now fix the values
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    g = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = g
    # Finally, take several photos with the fixed settings
    camera.capture_sequence(['images/image%02d.jpg' % i for i in range(10)])