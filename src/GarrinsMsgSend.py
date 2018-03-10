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

try:
    ser0 = serial.Serial('/dev/ttyUSB0', 115200)
except:
    ser0 = None

try:
    ser1 = serial.Serial('/dev/ttyUSB1', 115200)
except:
    ser1 = None
    
try:
    ser2 = serial.Serial('/dev/ttyUSB2', 115200)
except:
    ser2 = None
    
arduino_motor_drivers = [ser0, ser1, ser2]

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
    
def setSpeedSetpoint(ser):
    writeSerialData(ser, SET_SPEED, REQUEST, 200)
    return readSerialData(ser)

def main():
    while True:
        for driver in arduino_motor_drivers:
            if driver is not None:
                driverID = getDriverID(driver)
                print ('serial ID = '+str(driverID))
        
                driverFW = getFWVersion(driver)
                print ('serial FW Version = '+str(driverFW))
        
                driverUptime = getUptime(driver)
                print ('serial Uptime = '+str(driverUptime)+' ms')
                
                driverSetpoint = setSpeedSetpoint(driver)
                print ('serial setpoint = '+str(driverSetpoint))
        time.sleep(1)

if __name__ == "__main__":
    main()

