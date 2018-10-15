'''
Object detection and tracking with OpenCV
    ==> Turning a LED on detection and
    ==> Printing object position Coordinates 

    Based on original tracking object code developed by Adrian Rosebrock
    Visit original post: https://www.pyimagesearch.com/2016/05/09/opencv-rpi-gpio-and-gpio-zero-on-the-raspberry-pi/

Developed by Marcelo Rovai - MJRoBot.org @ 9Feb2018 
'''

# import the necessary packages
from __future__ import print_function
from imutils.video import VideoStream
from imutils.video import FPS
from multiprocessing import Process
from multiprocessing import Queue
import numpy as np
import imutils
import time
import cv2
import os
import RPi.GPIO as GPIO

# initialize LED GPIO
redLed = 21
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(redLed, GPIO.OUT)

def getcnts(inputQueue, outputQueue):
    colorLower = (11, 120, 100)
    colorUpper = (29, 255, 255)
    kernel = np.ones((5,5),np.uint8)
    while True:
        if not inputQueue.empty():
            frame = inputQueue.get()
            frame = imutils.resize(frame, width=500)
            #frame = imutils.rotate(frame, angle=180)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # construct a mask for the object color, then perform
            # a series of dilations and erosions to remove any small
            # blobs left in the mask
            mask = cv2.inRange(hsv, colorLower, colorUpper)
            mask = cv2.erode(mask, kernel, iterations=1)
            #mask = cv2.dilate(mask, None, iterations=3)
            # find contours in the mask and initialize the current
            # (x, y) center of the object
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = cnts[0] if imutils.is_cv2() else cnts[1]
            outputQueue.put(cnts)
            #center = None

# print object coordinates
def mapObjectPosition (x, y):
    print ("[INFO] Object Center coordenates at X0 = {0} and Y0 =  {1}".format(x, y))


inputQueue = Queue(maxsize=1)
outputQueue = Queue(maxsize=1)
cnts = None

p = Process(target=getcnts, args=(inputQueue,outputQueue,))
p.daemon = True
p.start()

# initialize the video stream and allow the camera sensor to warmup
print("[INFO] waiting for camera to warmup...")
vs = VideoStream(usePiCamera=True).start()
time.sleep(2.0)
fps = FPS().start()

# define the lower and upper boundaries of the object
# to be tracked in the HSV color space
# Start with LED off
GPIO.output(redLed, GPIO.LOW)
ledOn = False

# loop over the frames from the video stream
while True:
	# grab the next frame from the video stream, Invert 180o, resize the
	# frame, and convert it to the HSV color space
        frame = vs.read()

        if inputQueue.empty():
            inputQueue.put(frame)


        if not outputQueue.empty():
            cnts = outputQueue.get()

        # only proceed if at least one contour was found
        if cnts is not None and len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if (M['m00'] != 0):
                center = (int(M["m10"] / M["m00"] - 100), int(M["m01"] / M["m00"]-30))
            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x - 100), int(y-30)), int(radius),(0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                # position Servo at center of circle
                mapObjectPosition(int(x), int(y))
                # if the led is not already on, turn the LED on
                #if not ledOn:
                #	GPIO.output(redLed, GPIO.HIGH)
                #	ledOn = True
        # if the ball is not detected, turn the LED off
        #elif ledOn:
        #GPIO.output(redLed, GPIO.LOW)
        #ledOn = False
        # show the frame to our screen
        cv2.imshow("Frame", frame)
        fps.update()
        # if [ESC] key is pressed, stop the loop
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break

# do a bit of cleanup
print("\n [INFO] Exiting Program and cleanup stuff \n")
fps.stop()
print("[info] elasped time: {:.2f}".format(fps.elapsed()))
print('info FPS: {:.2f}'.format(fps.fps()))
GPIO.cleanup()
cv2.destroyAllWindows()
vs.stop()
