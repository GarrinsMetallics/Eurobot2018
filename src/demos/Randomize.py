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
GET_POSITION = 8
SET_POSITION = 9
GET_STATE = 10

REQUEST = 10
RESPONSE = 11

ROBOT_IDLE = 10
ROBOT_MOVING = 11
ROBOT_BLOCKED = 12

try:
    ser0 = serial.Serial('/dev/ttyUSB0', 115200, timeout=2.0)
except:
    ser0 = None

try:
    ser1 = serial.Serial('/dev/ttyUSB1', 115200, timeout=2.0)
except:
    ser1 = None
    
try:
    ser2 = serial.Serial('/dev/ttyUSB2', 115200, timeout=2.0)
except:
    ser2 = None

arduino_motor_drivers = [ser0, ser1, ser2]

def writeSerialData(ser, msg_cmd, msg_type, msg_value):
    # Write some data: 1 byte command + 4 bytes value
    ser.write(msg_cmd.to_bytes(1, byteorder='little'))
    ser.write(msg_type.to_bytes(1, byteorder='little'))
    ser.write(msg_value.to_bytes(4, byteorder='little', signed=True))
    
    time.sleep(0.01)
    
def readSerialData(ser):
    # Read reply from arduino: 1 byte command + 4 bytes value
    msg_cmd = int.from_bytes(ser.read(1), byteorder='little', signed=False)
    msg_type = int.from_bytes(ser.read(1), byteorder='little', signed=False)
    msg_value = int.from_bytes(ser.read(4), byteorder='little', signed=True)
    
    time.sleep(0.01)
    
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

def getPulses(ser):
    writeSerialData(ser, GET_STEPS, REQUEST, 0)
    return readSerialData(ser)
    
    
def getDistance(ser):
    writeSerialData(ser, GET_POSITION, REQUEST, 0)
    return readSerialData(ser)
    
def setDistance(ser, set_value):
    writeSerialData(ser, SET_POSITION, REQUEST, set_value)
    return readSerialData(ser)
    
def getState(ser):
    writeSerialData(ser, GET_STATE, REQUEST, 0)
    return readSerialData(ser)





    


def main():
    ##################################MOTOR########################
 
    for i in range (0,2):
        seguent=False
        vel1=E0()
        vel2=E1()
        vel3=E2()
        n=0      
        
        while seguent==False:
            if (ser0) is not None:
                
                if n==0:
                    driverDistance = setDistance(ser0, vel1[i])
                    print("motor1 valor",vel1[i])
                    #driverDistance = setDistance(ser1, vel2[i])
                    #driverDistance = setDistance(ser2, vel3[i])

                if getState(ser0)== ROBOT_IDLE :
                    seguent=True
                n=1


if __name__ == "__main__":
    main()




    







    
