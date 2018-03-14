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





def E0():
    motor1=200
    motor2=500
    motor3=0
    return motor1, motor2, motor3

def E1():
    motor1=0
    motor2=-500
    motor3=400
    
    return motor1,motor2,motor3

def E2():
    motor1=-200
    motor2=0
    motor3=-400
    
    
    return motor1, motor2, motor3
    
machine={
    0 : E0,
    1 : E1,
    2 : E2,
    }
    


def main():
    ##################################MOTOR1########################
    
    for cord in machine:
        seguent=False
        vel1,vel2,vel3=cord
        
        while seguent==False:
            if (ser0 and ser1 and ser2) is not None:
                if getState(driver) == ROBOT_IDLE:
                    print ('GARRINATOR V is IDLE.motor1')
                driverDistance = setDistance(driver, vel1)
                
                #while getState(driver) == ROBOT_MOVING:                
                   # print ('GARRINATOR V is moving.')
                    
                if getState(driver) == ROBOT_IDLE:
                    print ('GARRINATOR V is IDLE.motor1')

        ##################################MOTOR2########################
            
                if getState(driver) == ROBOT_IDLE:
                    print ('GARRINATOR V is IDLE.motor2')
                driverDistance = setDistance(driver, vel2)
                
                #while getState(driver) == ROBOT_MOVING:                
                   # print ('GARRINATOR V is moving.')
                    
                if getState(driver) == ROBOT_IDLE:
                    print ('GARRINATOR V is IDLE. motor2')
        ##################################MOTOR3########################
                    
                if getState(driver) == ROBOT_IDLE:
                    print ('GARRINATOR V is IDLE.motor3')
                driverDistance = setDistance(driver, vel3)
                
               # while getState(driver) == ROBOT_MOVING:                
                   # print ('GARRINATOR V is moving.')
                    
                if getState(driver) == ROBOT_IDLE:
                    print ('GARRINATOR V is IDLE.motor3')
                if getState(ser0)== ROBOT_IDLE and getState(ser1) == ROBOT_IDLE and getState(ser2) == ROBOT_IDLE:
                    seguent=True



if __name__ == "__main__":
    main()




    

        







    
