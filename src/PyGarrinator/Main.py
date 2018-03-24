import time
import serial
import math

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

OFF = 0
ON = 1

LAUNCH_BALLS = 10

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
            elif getDriverID(port) == 303:
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

def control_launcher(option):
    #set launcher gpio on/off
    time.sleep(0.1)

def read_coordinates():
    coordinates = []
    with open("coordinates.txt") as fp:
        for line in fp:
            coordinates.append([int(x) for x in line.split()])

    return coordinates

def get_direction(X,Y):
    if X == 0 and Y > 0:
        direction = "+CA"
    elif X == 0 and Y < 0:
        direction = "-CA"
    elif X > 0 and Y > 0:
        direction = "-AB"
    elif X > 0 and Y < 0:
        direction = "+BC"
    elif X < 0 and Y > 0:
        direction = "-BC"
    elif X < 0 and Y < 0:
        direction = "+AB"
    else:
        direction = "STOP"

    return direction

def get_D1(coordinate):
    X = coordinate[0]
    Y = coordinate[1]

    direction = get_direction(X, Y)

    D1 = int(X/math.cos(math.radians(30)))

    return D1, direction

def get_D2(coordinate):
    X = coordinate[0]
    Y = coordinate[1]

    direction = get_direction(X, Y)

    D2 = int(Y - X * math.tan(math.radians(30)))

    return D2, direction

def get_translation(distance, direction):
    transA = 0
    transB = 0
    transC = 0

    if direction == "+AB":
        transA = -distance
        transB = distance
    elif direction == "-AB":
        transA = distance
        transB = -distance
    elif direction == "+BC":
        transB = -distance
        transC = distance
    elif direction == "-BC":
        transB = distance
        transC = -distance
    elif direction == "+CA":
        transC = -distance
        transA = distance
    elif direction == "-CA":
        transC = distance
        transA = -distance

    return transA, transB, transC
    
def get_rotation(coordinate):
    angle = coordinate[2] % 360

    robot_perimeter = 2*math.pi*170

    rotation_distance = robot_perimeter*angle/359

    return rotation_distance

def get_action(coordinate):
    action = coordinate[3]

    return action

def robot_is_moving(A,B,C):
    movA = getState(A) == ROBOT_MOVING
    movB = getState(B) == ROBOT_MOVING
    movC = getState(C) == ROBOT_MOVING

    return movA and movB and movC

def main():
    # State 0: activate motors
    motorA, motorB, motorC = get_motors()

    # State 1: read coordinates list
    coordinates = read_coordinates()

    # For each coordinate
    for c in coordinates:

        # Move to new coordinate
        movD1, dirD1 = get_D1(c)
        movD2, dirD2 = get_D2(c)

        transA, transB, transC = get_translation(movD1, dirD1)
        setDistance(motorA, transA)
        setDistance(motorB, transB)
        setDistance(motorC, transC)

        # Wait for the robot to stop
        while robot_is_moving(motorA, motorB, motorC):
            time.sleep(0.5)

        transA, transB, transC = get_translation(movD2, dirD2)
        setDistance(motorA, transA)
        setDistance(motorB, transB)
        setDistance(motorC, transC)

        # Wait for the robot to stop
        while robot_is_moving(motorA, motorB, motorC):
            time.sleep(0.5)

        # Rotate the desired angle
        rotation = get_rotation(c)
        setDistance(motorA, rotation)
        setDistance(motorB, rotation)
        setDistance(motorC, rotation)

        # Wait for the robot to stop
        while robot_is_moving(motorA, motorB, motorC):
            time.sleep(0.5)

        action = get_action(c)
        if action == LAUNCH_BALLS:
            control_launcher(1)
            time.sleep(10)
            control_launcher(0)



if __name__ == "__main__":
    main()
