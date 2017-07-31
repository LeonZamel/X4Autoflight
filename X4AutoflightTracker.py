import serial
import time
import cv2
import numpy as np
import math
from picamera.array import PiRGBArray
from picamera import PiCamera
from imutils.video.pivideostream import PiVideoStream

# Establish connection to Arduino
ser = serial.Serial('/dev/ttyACM0', 9600)

center = (160, 120)
offset = (0, 0)

class Quadcopter():
    def __init__(self):
        self.position = center
        self.rotation = 0
        self.height = 0

params = cv2.SimpleBlobDetector_Params()
params.filterByArea = True
params.minArea = 10
params.maxArea = 50
blobDetector = cv2.SimpleBlobDetector_create(params)

kernel = np.ones((5, 5), np.uint8)
keyPoints = []

# HSV color boundaries for detecting colors
colorBoundaries = {
    # Detect Red in inverted picture, which is Cyan
    "red": ([50, 70, 70], [130, 253, 253]),
    
    "blue": ([40, 50, 60], [140, 253, 253])
}

"""
# Parts of image capturing code by http://www.pyimagesearch.com
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
# camera.crop = (0.0, 0.0, 0.7, 0.7)
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
 
# allow the camera to warmup
time.sleep(0.1)
"""
speed = 30

searchsize = 50

quad = Quadcopter()
quad.position = center

stream = PiVideoStream(framerate=30)
stream.camera.zoom= (0.0, 0.0, 0.6, 0.6)
stream.start()

time.sleep(0.1)

x = 0
y = 0

last_cmd = 0

vel = (0, 0)

throttle = 120

while True:
    image = stream.read()
    if image is not None:
        testimg = image[quad.position[1] - searchsize : quad.position[1] + searchsize, quad.position[0] - searchsize : quad.position[0] + searchsize]
        #cv2.imshow("region", testimg)
        hsvImage = cv2.cvtColor(testimg, cv2.COLOR_BGR2HSV)
        hsvInvImage = np.zeros(hsvImage.shape)
        hsvInvImage = cv2.cvtColor(cv2.bitwise_not(testimg), cv2.COLOR_BGR2HSV)
        # dispImg = np.zeros(image.shape)
        """
        # capture frames from the camera
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            # grab the raw NumPy array representing the image, then initialize the timestamp
            # and occupied/unoccupied text
            image = frame.array
            dispImg = np.zeros(image.shape)

            # analyze image for quad:
            # loop over the boundaries
            """
        for color in colorBoundaries:
            # create NumPy arrays from the boundaries
            lower = np.array(colorBoundaries[color][0], dtype="uint8")
            upper = np.array(colorBoundaries[color][1], dtype="uint8")
        
            # find the colors within the specified boundaries and apply
            # the mask
            if color == "red":
                output = cv2.inRange(hsvInvImage, lower, upper)
            else:
                output = cv2.inRange(hsvImage, lower, upper)

            # output = cv2.dilate(output, kernel, iterations = 1)
            
            #output = cv2.erode(output, kernel, iterations = 1)
            # output= cv2.medianBlur(output, 7)
            output= cv2.GaussianBlur(output, (5, 5), 0)
            #output = cv2.dilate(output, kernel, iterations = 1)
            output= cv2.medianBlur(output, 5)
            # cv2.bitwise_not(output, output)

            keyPoints.append(blobDetector.detect(output))

            # show the images
            cv2.imshow(color, output)

        oldPos = quad.position
        oldHeight = quad.height

        if len(keyPoints) == 2:
            if all([len(pair) == 2 for pair in keyPoints]):
                frontMid = [0, 0]
                backMid = [0, 0]

                frontMid = (int((keyPoints[1][0].pt[0] + keyPoints[1][1].pt[0]) / 2), int((keyPoints[1][0].pt[1] + keyPoints[1][1].pt[1]) / 2))
                backMid = (int((keyPoints[0][0].pt[0] + keyPoints[0][1].pt[0]) / 2), int((keyPoints[0][0].pt[1] + keyPoints[0][1].pt[1]) / 2))

                quad.height = (abs(keyPoints[1][0].pt[0] - keyPoints[1][1].pt[0]) + abs(keyPoints[0][0].pt[0] - keyPoints[0][1].pt[0]) + abs(keyPoints[0][0].pt[1] - keyPoints[0][1].pt[1]) + abs(keyPoints[0][0].pt[1] - keyPoints[0][1].pt[1]) / 2)
                               
                quad.position = (int((frontMid[0] + backMid[0]) / 2) + quad.position[0] - searchsize, int((frontMid[1] + backMid[1]) / 2) + quad.position[1] - searchsize)
                quad.rotation = math.degrees(math.atan2(backMid[1] - frontMid[1], backMid[0] - frontMid[0])) -90

        """
        if len(keyPoints) == 3:
            if len(keyPoints[0]) == 2:
                # both red points found
                backMid = (int((keyPoints[0][0].pt[0] + keyPoints[0][1].pt[0]) / 2), int((keyPoints[0][0].pt[1] + keyPoints[0][1].pt[1]) / 2))
                lineAngle = math.atan2(keyPoints[0][0].pt[1] - keyPoints[0][1].pt[1], keyPoints[0][0].pt[0] - keyPoints[0][1].pt[0])
        """
            
        # print(quad.position, quad.rotation)
        cv2.circle(image, quad.position, 3, (0, 255, 0), -1)
        cv2.line(image, quad.position, (int(quad.position[0] + math.cos(math.radians(quad.rotation)) * 30), int(quad.position[1] + math.sin(math.radians(quad.rotation)) * 30)), (0, 255, 0), 1)
                   
        # pts = np.float([keyPoints[idx].pt for idx in len(keyPoints)]).reshape(-1, 1, 2)
        
        # print(keyPoints)
        for pair in keyPoints:
            for point in pair:
                cv2.circle(image, (int(point.pt[0]), int(point.pt[1])), 10, (0, 255, 0))
        

        cv2.circle(image, center, 5, (255, 255, 0), -1)

        # beforeOffset = offset

        offset = [center[0] - quad.position[0], center[1] - quad.position[1]]
        # print(offset)

        lastVel = vel

        vel = (quad.position[0] - oldPos[0], quad.position[1] - oldPos[1])
        
        x = 0
        y = 0
        

        """
        if np.any(beforeOffset != offset):
            if beforeOffset[0] != offset[0]:
                x = offset[0] * -speed
            if beforeOffset[1] != offset[1]:
                y = offset[1] * speed
        """
        
        """
        if time.time() - last_cmd > 0.5:
            last_cmd = time.time()
            if offset[0] < 0:
                # Quad to the right
                x = speed
                
            else:
                x = -speed

            if offset[1] < 0:
                # Quad to the bottom
                y = -speed
            else:
                y = speed
        """

        #print(vel)
        #print(quad.rotation)
        
        #ser.write(bytearray([0, 0, int(130),  int(130)]))
        #ser.write(bytearray([int(255 - 0.8 * quad.height) - 5, int(132 - quad.rotation), int(133 - offset[0] / 3 + (lastVel[0] + vel[0] * 2) / 3), int(130 + offset[1] / 3 - (lastVel[1] + vel[1] * 2) / 3)]))
        valX = int(130 - offset[0] / 2 + vel[0] * 3)
        valY = int(130 + offset[1] / 2 - vel[1] * 3)

        if valX > 130:
            setX = min(valX, 160)
        else:
            setX = max(valX, 100)

        if valY > 130:
            setY = min(valY, 160)
        else:
            setY = max(valY, 100)

        heightacc = quad.height - oldHeight
        if heightacc > -0.5:
            heightacc = max(heightacc, 0.5) * 2
        else:
            heightacc = min(heightacc, -0.5) * 2


        if quad.height < 35:
            throttle += 1
        elif quad.height < 44:
            throttle += 0.5 / (heightacc / 4)
        elif quad.height < 47:
            throttle += 0.25 / (heightacc / 2)
        elif quad.height < 50:
            throttle += 0.125 / heightacc
        elif quad.height < 55:
            throttle -= heightacc / 4
        elif quad.height > 62:
            throttle -= heightacc / 2
        elif quad.height > 55:
            throttle -= heightacc / 3

        throttle = max(throttle, 125)
        throttle = min(throttle, 220)

        print(quad.height)
        print(throttle)

        ser.write(bytearray([int(throttle), int(132 - quad.rotation), setX, setY]))
        print(setX, setY)
        # print(int(-offset[0] / 10 + 163), int(offset[1] / 10 + 163))

        keyPoints = []

        # show the frame
        # cv2.imshow("Detection", dispImg)
        cv2.imshow("Frame", image)
            
        
        # cv2.imshow("Detected", detectedBlobImage)
        key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            ser.close()
            break
