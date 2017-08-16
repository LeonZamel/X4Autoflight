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
params.minArea = 5
params.maxArea = 50
blobDetector = cv2.SimpleBlobDetector_create(params)

kernel = np.ones((5, 5), np.uint8)
keyPoints = []

# HSV color boundaries for detecting colors
colorBoundaries = {
    # Detect Red in inverted picture, which is Cyan
    "red": ([60, 70, 80], [130, 253, 253]),
    
    "blue": ([50, 55, 60], [135, 253, 253])
}

searchsize = 45

quad = Quadcopter()
quad.position = center

stream = PiVideoStream(framerate=30)
stream.camera.zoom= (0.0, 0.0, 0.6, 0.6)
stream.start()

time.sleep(0.1)

vel = (0, 0)

while True:
    image = stream.read()
    if image is not None:
        testimg = image[quad.position[1] - searchsize : quad.position[1] + searchsize, quad.position[0] - searchsize : quad.position[0] + searchsize]
        hsvImage = cv2.cvtColor(testimg, cv2.COLOR_BGR2HSV)
        hsvInvImage = np.zeros(hsvImage.shape)
        hsvInvImage = cv2.cvtColor(cv2.bitwise_not(testimg), cv2.COLOR_BGR2HSV)

        for color in colorBoundaries:
            # Create NumPy arrays from the boundaries
            lower = np.array(colorBoundaries[color][0], dtype="uint8")
            upper = np.array(colorBoundaries[color][1], dtype="uint8")
        
            # Find the colors within the specified boundaries and apply the mask
            if color == "red":
                output = cv2.inRange(hsvInvImage, lower, upper)
            else:
                output = cv2.inRange(hsvImage, lower, upper)

            if int(quad.height / 20) % 2 == 0:
                fsize= int(quad.height / 20) + 3
            else: 
                fsize = int(quad.height / 20) + 2

            fsize = 5
                
            output= cv2.GaussianBlur(output, (fsize,fsize), 0)
            output= cv2.medianBlur(output, fsize)

            keyPoints.append(blobDetector.detect(output))

            # Show the images
            cv2.imshow(color, output)

        oldPos = quad.position
        oldHeight = quad.height

        # Calculate popsition
        if len(keyPoints) == 2:
            if all([len(pair) == 2 for pair in keyPoints]):
                frontMid = [0, 0]
                backMid = [0, 0]

                frontMid = (int((keyPoints[1][0].pt[0] + keyPoints[1][1].pt[0]) / 2), int((keyPoints[1][0].pt[1] + keyPoints[1][1].pt[1]) / 2))
                backMid = (int((keyPoints[0][0].pt[0] + keyPoints[0][1].pt[0]) / 2), int((keyPoints[0][0].pt[1] + keyPoints[0][1].pt[1]) / 2))

                quad.height = (abs(keyPoints[1][0].pt[0] - keyPoints[1][1].pt[0]) + abs(keyPoints[0][0].pt[0] - keyPoints[0][1].pt[0]) + abs(keyPoints[0][0].pt[1] - keyPoints[0][1].pt[1]) + abs(keyPoints[0][0].pt[1] - keyPoints[0][1].pt[1]) / 2)
                               
                quad.position = (int((frontMid[0] + backMid[0]) / 2) + quad.position[0] - searchsize, int((frontMid[1] + backMid[1]) / 2) + quad.position[1] - searchsize)
                quad.rotation = math.degrees(math.atan2(backMid[1] - frontMid[1], backMid[0] - frontMid[0])) -90

        # Draw stuff
        cv2.circle(image, quad.position, 3, (0, 255, 0), -1)
        cv2.line(image, quad.position, (int(quad.position[0] + math.cos(math.radians(quad.rotation)) * 30), int(quad.position[1] + math.sin(math.radians(quad.rotation)) * 30)), (0, 255, 0), 1)
        cv2.circle(image, center, 5, (255, 255, 0), -1)     
        for pair in keyPoints:
            for point in pair:
                cv2.circle(image, (int(point.pt[0]), int(point.pt[1])), 10, (0, 255, 0))
        
        
        offset = [center[0] - quad.position[0], center[1] - quad.position[1]]
        lastVel = vel
        vel = (quad.position[0] - oldPos[0], quad.position[1] - oldPos[1])
        
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

        # TODO: Add throttle control
        ser.write(bytearray([0, int(132 - quad.rotation), setX, setY]))
        print(setX, setY)
        keyPoints = []
        
        cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF

        # If the 'q' key was pressed, break from the loop
        if key == ord("q"):
            ser.close()
            break
