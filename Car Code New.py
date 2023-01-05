import RPi.GPIO as GPIO
import time
import RPi.GPIO as GPIO
from picamera import PiCamera
from PIL import Image
import numpy as np
from gpiozero import LED
import threading
import multiprocessing as mp
cam = PiCamera()
cam.resolution = (300, 300)
cam.framerate = 4
relay = LED(18)
led_red = LED(17)
# 140, 170
lo, hi = 140, 170
lo = int((lo * 255) / 360)
hi = int((hi * 255) / 360)


# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

# set GPIO Pins
GPIO_TRIGGER = 27
GPIO_ECHO = 24

# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)


def distance():
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
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
    return distance


def green_counter(q):
    # Open image and make RGB and HSV versions
    lo, hi = 140, 170
    lo = int((lo * 255) / 360)
    hi = int((hi * 255) / 360)
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

def camera():
    cam.capture("1.png")
    time.sleep(0.1)


while 1:
    #camera()
    w = threading.Thread(target=camera)		
    w.start()
    w.join()
    queue = mp.Queue()
    p = threading.Thread(target=green_counter, args=(queue,))
    p.start()
    green_pixels = queue.get()
    p.join()
    while green_pixels > 50000:
        dist = distance()
        #print(dist)
        led_red.off()
        relay.on()
        aer = True
        
        while aer == True:
            if dist <= 10:
                relay.off()
                led_red.on()
                aer = False
                green_pixels = 0
                break
            else:
                dist = distance()






