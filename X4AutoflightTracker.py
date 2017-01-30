import serial
import time

ser = serial.Serial('/dev/ttyACM0', 9600)
val = 0
while True:
    if val < 255:
        val+=1
    else:
        val = 0
    ser.write(b'200')
    time.sleep(0.1)
