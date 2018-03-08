#!/usr/bin/python

import time
import serial

DRIVER_ID = 1
FW_VERSION = 2
RUNNING_TIME = 3
GET_SPEED = 4
SET_SPEED = 5
GET_STEPS = 6
SET_STEPS = 7

REQUEST = 10
RESPONSE = 11

ser0 = serial.Serial('/dev/ttyUSB0', 9600)
#ser1 = serial.Serial('/dev/ttyUSB1', 9600)
#ser2 = serial.Serial('/dev/ttyUSB2', 9600)

def writeSerialData(ser, msg_cmd, msg_type, msg_value):
    # Write some data: 1 byte command + 4 bytes value
    ser.write(msg_cmd.to_bytes(1, byteorder='little'))
    ser.write(msg_type.to_bytes(1, byteorder='little'))
    ser.write(msg_value.to_bytes(4, byteorder='little', signed=True))
    
def readSerialData(ser):
    # Read reply from arduino: 1 byte command + 4 bytes value
    msg_cmd = int.from_bytes(ser.read(1), byteorder='little', signed=False)
    msg_type = int.from_bytes(ser.read(1), byteorder='little', signed=False)
    msg_value = int.from_bytes(ser.read(4), byteorder='little', signed=True)
    
    return msg_value

def getDriverID(ser):
    writeSerialData(ser, DRIVER_ID, REQUEST, 0)
    return readSerialData(ser)

def getFWVersion(ser):
    writeSerialData(ser, FW_VERSION, REQUEST, 0)
    return readSerialData(ser)

def getUptime(ser):
    writeSerialData(ser, RUNNING_TIME, REQUEST, 0)
    return readSerialData(ser)

def main():
    while True:
        ser0ID = getDriverID(ser0)
        print ('serial0 ID = '+str(ser0ID))
        
        ser0FW = getFWVersion(ser0)
        print ('serial0 FW Version = '+str(ser0FW))
        
        ser0Uptime = getUptime(ser0)
        print ('serial0 Uptime = '+str(ser0Uptime)+' ms')
        
        time.sleep(1)

if __name__ == "__main__":
    main()

