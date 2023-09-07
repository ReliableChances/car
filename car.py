import RPi.GPIO as GPIO
from gpiozero import AngularServo
from picamera import PiCamera
from gpiozero import LED
from PIL import Image
import numpy as np
import threading
import multiprocessing as mp
import time
servo = AngularServo(7, min_pulse_width=0.0006, max_pulse_width=0.0023)
servo.angle = 25
cam = PiCamera()
cam.resolution = (325, 325)
cam.framerate = 4

relay = LED(18)
led_red = LED(17)
# 140, 170
lo, hi = 140, 170
lo = int((lo * 255) / 360)
hi = int((hi * 255) / 360)
queue = mp.Queue()
		
# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

# set GPIO Pins
GPIO_TRIGGER = 27
GPIO_ECHO = 24

# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def get_distance():
    distance = 100

    while distance > 30:
            
            # set Trigger to HIGH
            GPIO.output(GPIO_TRIGGER, True)

            # set Trigger after 0.01ms to LOW
            time.sleep(0.00001)
            GPIO.output(GPIO_TRIGGER, False)

            StartTime = time.time()
            StopTime = time.time()

            # save StartTime
            while GPIO.input(GPIO_ECHO) == 0:
                StartTime = time.time()

            # save time of arrival
            while GPIO.input(GPIO_ECHO) == 1:
                StopTime = time.time()
            time.sleep(0.03)
            # time difference between start and arrival
            TimeElapsed = StopTime - StartTime
            # multiply with the sonic speed (34300 cm/s)
            # and divide by 2, because there and back
            distance = (TimeElapsed * 34300) / 2

            #print(distance)
  
        
    relay.off()
    led_red.on()
    servo.angle = -45
    time.sleep(1.5)
    servo.angle = 25
    time.sleep(0.3)
    

def green_counter(q):
    # Open image and make RGB and HSV versionsf    
    RGBim = Image.open("1.png").convert('RGB')
    HSVim = RGBim.convert('HSV')
    # Make numpy versions
    RGBna = np.array(RGBim)
    HSVna = np.array(HSVim)
    # Extract Hue
    H = HSVna[:, :, 0]
    green = np.where((H > lo) & (H < hi))
    RGBna[green] = [121, 254, 12]
    count = green[0].size
    #print("Green pixels: {}".format(count))
    #Image.fromarray(RGBna).save('result.png')
    q.put(count)
    

def take_photo():
    cam.capture("1.png")

# Set the camera frame rate




o = 1
while o == 1:
    #camera()
    w = threading.Thread(target=take_photo)
    w.start()
    w.join()
    p = threading.Thread(target=green_counter, args=(queue,))
    p.start()
    result = queue.get()
    p.join()
    #print(result)
    if result > -1:
        led_red.off()
        #time.sleep(0.35)
        relay.on()
        r = threading.Thread(target=get_distance)
        r.start()
        r.join()
        pass
        o = 1
            
                
            


    
      

