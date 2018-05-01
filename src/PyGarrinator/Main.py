import time
import serial
import math
import RPi.GPIO as GPIO

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
        ser0 = serial.Serial('/dev/ttyUSB0', 250000, timeout=1.0)
    except:
        ser0 = None

    try:
        ser1 = serial.Serial('/dev/ttyUSB1', 250000, timeout=1.0)
    except:
        ser1 = None
        
    try:
        ser2 = serial.Serial('/dev/ttyUSB2', 250000, timeout=1.0)
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
    
def setSpeedSetpoint(ser, set_value):
    writeSerialData(ser, SET_SPEED, REQUEST, set_value)
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

def robot_is_moving(A,B,C):
    movA = getState(A) == ROBOT_MOVING
    movB = getState(B) == ROBOT_MOVING
    movC = getState(C) == ROBOT_MOVING

    return movA or movB or movC

def get_mov(coordinate):
    X = coordinate[0]
    Y = coordinate[1]

    distance = math.sqrt(X*X+Y*Y)
    if Y == 0:
        if X > 0:
            angle = 0 #up
        else:
            angle = math.pi #down
    elif X == 0:
        if Y > 0:
            angle = 3*math.pi/2 #right
        else:
            angle = math.pi/2 #left
    else:
        angle = math.atan(Y/X)
    print('distance',distance)
    print('angle',angle)

    speedC = int(round(distance*math.sin((math.pi/2)-angle)))
    speedA = int(round(distance*math.sin((7*math.pi/6)-angle)))
    speedB = int(round(distance*math.sin((11*math.pi/6)-angle)))
    speedB = speedB*(-1)

    if X<0 and Y<=0:
        speedB = speedB*(-1)
        speedA = speedA*(-1)
        speedC = speedC*(-1)
    
    if X == 0 and Y!=0:
        speedB = speedB*(-1)
        speedA = speedA*(-1)

    if Y == 0 and X!=0:
        speedB = speedB*(-1)
        speedA = speedA*(-1)
        speedC = speedC*(-1)

    print('SpeedA: ',speedA)
    print('SpeedB: ',speedB)
    print('SpeedC: ',speedC)

    return speedA, speedB, speedC

def get_rotation(coordinate):
    angle = coordinate[2] 

    robot_perimeter = 2*math.pi*155

    rotation_distance = int(-robot_perimeter*angle/360)

    return rotation_distance

def get_action(coordinate):
    action = coordinate[3]

    return action

def trigger():
    GPIO.output(16,GPIO.HIGH)
    GPIO.output(20,GPIO.HIGH)
    GPIO.output(21,GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(16,GPIO.LOW)
    GPIO.output(20,GPIO.LOW)
    GPIO.output(21,GPIO.LOW)

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(16, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(20, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(21, GPIO.OUT, initial=GPIO.LOW)
    # State 0: activate motors
    motorA, motorB, motorC = get_motors()

    # State 1: read coordinates list
    coordinates = read_coordinates()
    
    # For each coordinate
    for c in coordinates:

        print(c)
        speedA, speedB, speedC = get_mov(c)
        
        setDistance(motorA, speedA)
        setDistance(motorB, speedB)
        setDistance(motorC, speedC)
        setSpeedSetpoint(motorA,speedA)
        setSpeedSetpoint(motorB,speedB)
        setSpeedSetpoint(motorC,speedC)
        trigger()

        while robot_is_moving(motorA, motorB, motorC):
            print('Is moving')
            time.sleep(1) 

        rotation = get_rotation(c)
        setDistance(motorA, rotation)
        setDistance(motorB, -rotation)
        setDistance(motorC, rotation)
        setSpeedSetpoint(motorA,300)
        setSpeedSetpoint(motorB,300)
        setSpeedSetpoint(motorC,-300)
        trigger()

        # Wait for the robot to stop
        while robot_is_moving(motorA, motorB, motorC):
            print('Is rotating')
            time.sleep(1)

        action = get_action(c)
        if action == LAUNCH_BALLS:
            control_launcher(1)
            time.sleep(10)
            control_launcher(0)
            trigger()
            

if __name__ == "__main__":
    main()
