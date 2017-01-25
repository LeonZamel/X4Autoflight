import serial
ser = serial.Serial('/dev/ttyACM0', 9600)
while True :
    try:
        state=ser.readline()
        print(state)
    except:
        pass
