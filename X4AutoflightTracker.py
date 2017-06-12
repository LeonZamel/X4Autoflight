import serial
import time
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera

# Establish connection to Arduino
# ser = serial.Serial('/dev/ttyACM0', 9600)

params = cv2.SimpleBlobDetector_Params()
params.filterByArea = True
params.minArea = 20
params.maxArea = 10000
blobDetector = cv2.SimpleBlobDetector_create(params)

kernel = np.ones((15, 15), np.uint8)
keyPoints = []

# BGR color boundaries for detecting colors
colorBoundaries = {
    "red": ([80, 80, 180], [220, 240, 255]),
    "blue": ([180, 60, 60], [255, 230, 190])
}

# Parts of image capturing code by http://www.pyimagesearch.com
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
 
# allow the camera to warmup
time.sleep(0.1)
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array


    # analyze image for quad:
    # loop over the boundaries
    
    for color in colorBoundaries:
        # create NumPy arrays from the boundaries
        lower = np.array(colorBoundaries[color][0], dtype="uint8")
        upper = np.array(colorBoundaries[color][1], dtype="uint8")
    
        # find the colors within the specified boundaries and apply
        # the mask
        output= cv2.inRange(image, lower, upper)

        output= cv2.medianBlur(output, 11)
        
        output = cv2.dilate(output, kernel, iterations = 1)
        
        output = cv2.erode(output, kernel, iterations = 2)

        output= cv2.medianBlur(output, 9)
        
        cv2.bitwise_not(output, output)


        keyPoints.extend(blobDetector.detect(output))

        # show the images
        cv2.imshow(color, output)

    

    # pts = np.float([keyPoints[idx].pt for idx in len(keyPoints)]).reshape(-1, 1, 2)
    print(keyPoints)
    for point in keyPoints:
        cv2.circle(image, (int(point.pt[0]), int(point.pt[1])), 10, (0, 255, 0))

    keyPoints = []

    # show the frame
    cv2.imshow("Frame", image)
        
    
    # cv2.imshow("Detected", detectedBlobImage)
    key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
