'''
Object detection and tracking with OpenCV
    ==> Turning a LED on detection and
    ==> Real Time tracking with Pan-Tilt servos

    Based on original tracking object code developed by Adrian Rosebrock
    Visit original post: https://www.pyimagesearch.com/2016/05/09/opencv-rpi-gpio-and-gpio-zero-on-the-raspberry-pi/

Developed by Marcelo Rovai - MJRoBot.org @ 9Feb2018
'''

# import the necessary packages
from __future__ import print_function
from imutils.video import VideoStream
from multiprocessing import Process
from multiprocessing import Queue
import argparse
import imutils
import time
import cv2
import os
import RPi.GPIO as GPIO
import numpy as np
from time import sleep

#define Servos GPIOs
panServo = 23
tiltServo = 24
previouspos = 0
#prepos = 0
# initiliza process
inputQueue = Queue(maxsize=1)
outputQueue = Queue(maxsize=1)
cnts = None
def getcnts(inputQueue, outputQueue):

    # define the lower and upper boundaries of the object
    # to be tracked in the HSV color space
    colorLower = (11, 120, 100)
    colorUpper = (29, 255, 255)
    kernel = np.ones((5,5),np.uint8)
    while True:
        # grab the next frame from the video stream, Invert 180o, resize the
        # frame, and convert it to the HSV color space
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
            #mask = cv2.dilate(mask, None, iterations=2)
            # find contours in the mask and initialize the current
            # (x, y) center of the object
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)
            cnts = cnts[0] if imutils.is_cv2() else cnts[1]
            center = None
            outputQueue.put(cnts)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(servo, GPIO.OUT)

def setServoAngle(servo, angle):
	#assert angle >=30 and angle <= 150
	pwm = GPIO.PWM(servo, 50)
	pwm.start(8)
	dutyCycle = angle / 20. + 2.
	pwm.ChangeDutyCycle(dutyCycle)
	sleep(0.4)
	pwm.stop()
    print("[INFO] Positioning servo at GPIO {0} to {1} degrees\n".format(servo, angle))

#position servos
def positionServo (servo, angle):
    #os.system("python angleServoCtrl.py " + str(servo) + " " + str(angle))
    print("[INFO] Positioning servo at GPIO {0} to {1} degrees\n".format(servo, angle))

# position servos to present object at center of the frame
def mapServoPosition (x, y):
    global panAngle
    global tiltAngle

    k = 0.1
    panAngle = 90 - k * (250-x)
    panAngle = int(panAngle)
    setServoAngle (panServo, panAngle)

    #if (x < 300):
    #    panAngle -= 10
    #    if panAngle < 60:
    #        panAngle = 60
    #    positionServo (panServo, panAngle)

    #if (x > 200):
    #    panAngle += 10
    #    if panAngle > 120:
    #        panAngle = 120
    #    positionServo (panServo, panAngle)

    if (y < 160):
        tiltAngle -= 10
        if tiltAngle < 40:
            tiltAngle = 40
        #positionServo (tiltServo, tiltAngle)

    if (y > 210):
        tiltAngle += 10
        if tiltAngle < 140:
            tiltAngle = 140
        #positionServo (tiltServo, tiltAngle)

p = Process(target=getcnts, args=(inputQueue,outputQueue,))
p.daemon=True
p.start()

# initialize the video stream and allow the camera sensor to warmup
print("[INFO] waiting for camera to warmup...")
vs = VideoStream(usePiCamera=True).start()
time.sleep(2.0)


# Initialize angle servos at 90-90 position
global panAngle
panAngle = 90
global tiltAngle
tiltAngle =90

# positioning Pan/Tilt servos at initial position
positionServo (panServo, panAngle)
positionServo (tiltServo, tiltAngle)

# loop over the frames from the video stream
while True:

    frame = vs.read()

    if inputQueue.empty():
        inputQueue.put(frame)

    if not outputQueue.empty():
        cnts = outputQueue.get()

    if cnts is not None:
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            #M = cv2.moments(c)
            #center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 5 and radius < 25:
                # draw the circle and centroid on the frame
                # then update the list of tracked points
                cv2.circle(frame, (int(x-100), int(y-30)), int(radius),
                        (0, 255, 255), 2)
                #cv2.circle(frame, center, 5, (0, 0, 255), -1)
                # position Servo at center of circle
                mapServoPosition(int(x-100), int(y-30))
                #previouspos = int(x-100)
        # show the frame to our screen
        cv2.imshow("Frame", frame)
        # if [ESC] key is pressed, stop the loop
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break

# do a bit of cleanup
print("\n [INFO] Exiting Program and cleanup stuff \n")
positionServo (panServo, 90)
positionServo (tiltServo, 90)
GPIO.cleanup()
cv2.destroyAllWindows()
vs.stop()
