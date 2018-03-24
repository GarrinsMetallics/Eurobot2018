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

def get_motors():
    try:
        ser0 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1.0)
    except:
        ser0 = None

    try:
        ser1 = serial.Serial('/dev/ttyUSB1', 115200, timeout=1.0)
    except:
        ser1 = None
        
    try:
        ser2 = serial.Serial('/dev/ttyUSB2', 115200, timeout=1.0)
    except:
        ser2 = None

    ports = [ser0, ser1, ser2]

    motorA = None
    motorB = None
    motorC = None

    for port in ports:
        if port is not None:
            if getDriverID(port) == 101:
                motorA = port
            elif getDriverID(port) == 202:
                motorB = port
            elif getDriverID(driver) == 303:
                motorC = port
                
    return motorA, motorB, motorC

def writeSerialData(ser, msg_cmd, msg_type, msg_value):
    if ser is not None:
        # Write some data: 1 byte command + 4 bytes value
        ser.write(msg_cmd.to_bytes(1, byteorder='little'))
        ser.write(msg_type.to_bytes(1, byteorder='little'))
        ser.write(msg_value.to_bytes(4, byteorder='little', signed=True))
    
        time.sleep(0.01)
    
def readSerialData(ser):
    if ser is not None:
       # Read reply from arduino: 1 byte command + 4 bytes value
       msg_cmd = int.from_bytes(ser.read(1), byteorder='little', signed=False)
       msg_type = int.from_bytes(ser.read(1), byteorder='little', signed=False)
       msg_value = int.from_bytes(ser.read(4), byteorder='little', signed=True)
          
       time.sleep(0.01)
    
       return msg_value
    else:
        return -1

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

def read_coordinates():
    coordinates = []
    with open("coordinates.txt") as fp:
        for line in fp:
            coordinates.append([int(x) for x in line.split()])

    return coordinates

def get_translations(coordinate):
    X = coordinate[0]
    Y = coordinate[1]

    return 0,0,0
    
def get_rotations(coordinate):
    angle = coordinate[2]
    angle = angle % 360

    return 0,0,0

def main():
    motorA, motorB, motorC = get_motors()
    coordinates = read_coordinates()
    for c in coordinates:
        transA,transB,transC = get_translations(c)
        setDistance(motorA,transA)
        setDistance(motorB,transB)
        setDistance(motorC,transC)
        while getState(motorA) == MOVING and getState(motorB) == MOVING and getState(motorC) == MOVING:
            time.sleep(0.5)

        rotA,rotB,rotC = get_rotations(c)

        setDistance(motorA,rotA)
        setDistance(motorB,rotB)
        setDistance(motorC,rotC)

        while getState(motorA) == MOVING and getState(motorB) == MOVING and getState(motorC) == MOVING:
            time.sleep(0.5)

        

if __name__ == "__main__":
    main()
