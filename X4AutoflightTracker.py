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

class Quadcopter():
    def __init__(self):
        self.position = (0, 0)
        self.rotation = 0

center = (160, 120)
offset = (0, 0)

params = cv2.SimpleBlobDetector_Params()
params.filterByArea = True
params.minArea = 3
params.maxArea = 100
blobDetector = cv2.SimpleBlobDetector_create(params)

kernel = np.ones((3, 3), np.uint8)
keyPoints = []

# HSV color boundaries for detecting colors
colorBoundaries = {
    # Detect Red in inverted picture, which is Cyan
    "red": ([50, 80, 20], [130, 255, 255]),
    
    "blue": ([60, 80, 50], [160, 255, 255])
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

while True:
    image = stream.read()
    if image is not None:
        hsvImage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsvInvImage = np.zeros(hsvImage.shape)
        hsvInvImage = cv2.cvtColor(cv2.bitwise_not(image), cv2.COLOR_BGR2HSV)
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
            
            output = cv2.erode(output, kernel, iterations = 1)
            # output= cv2.medianBlur(output, 7)
            output= cv2.GaussianBlur(output, (3, 3), 0)
            #output = cv2.dilate(output, kernel, iterations = 1)
            output= cv2.medianBlur(output, 3)
            # cv2.bitwise_not(output, output)

            keyPoints.extend(blobDetector.detect(output))

            # show the images
            #cv2.imshow(color, output)

        oldPos = quad.position

        if len(keyPoints) == 4:

            frontMid = [0, 0]
            backMid = [0, 0]

            frontMid = (int((keyPoints[0].pt[0] + keyPoints[1].pt[0]) / 2), int((keyPoints[0].pt[1] + keyPoints[1].pt[1]) / 2))
            backMid = (int((keyPoints[2].pt[0] + keyPoints[3].pt[0]) / 2), int((keyPoints[2].pt[1] + keyPoints[3].pt[1]) / 2))

            quad.position = (int((frontMid[0] + backMid[0]) / 2), int((frontMid[1] + backMid[1]) / 2))
            quad.rotation = math.atan2(backMid[1] - frontMid[1], backMid[0] - frontMid[0])
            
        # print(quad.position, quad.rotation)
        cv2.circle(image, quad.position, 5, (0, 255, 0), -1)
        cv2.line(image, quad.position, (int(quad.position[0] + math.cos(quad.rotation) * 30), int(quad.position[1] + math.sin(quad.rotation) * 30)), (0, 255, 0), 2)
                   
        # pts = np.float([keyPoints[idx].pt for idx in len(keyPoints)]).reshape(-1, 1, 2)
        
        # print(keyPoints)
        
        for point in keyPoints:
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

        print(vel)
        
        #ser.write(bytearray([0, 0, int(130),  int(130)]))
        ser.write(bytearray([0, 0, int(130 - offset[0] / 3 + (lastVel[0] + vel[0])), int(130 + offset[1] / 3 - (lastVel[1] + vel[1]))]))
            
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
