import serial
import time
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera

# Establish connection to Arduino
# ser = serial.Serial('/dev/ttyACM0', 9600)


# BGR color boundaries for detecting colors
colorBoundaries = {
    "red": ([0, 0, 100], [100, 100, 255]),
    "blue": ([100, 0, 0], [255, 100, 100])
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
        mask = cv2.inRange(image, lower, upper)
        output = cv2.bitwise_and(image, image, mask=mask)
        # show the images
        cv2.imshow(color, output)

    # show the frame
    cv2.imshow("Frame", image)
    key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
