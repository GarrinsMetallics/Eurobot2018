import threading
import time

from collections import deque
import numpy as np
import imutils
import cv2

import time
import serial
import math
import RPi.GPIO as GPIO

DRIVER_ID=1
FW_VERSION=2
RUNNING_TIME=3
GET_SPEED=4
SET_SPEED=5
GET_STEPS=6
SET_STEPS=7
GET_POSITION=8
SET_POSITION=9
GET_STATE=10
STOP=11
RESTART=12

REQUEST=10
RESPONSE=11

ROBOT_IDLE=10
ROBOT_MOVING=11
ROBOT_BLOCKED=12

OFF=0
ON=1

LAUNCH_BALLS=10
PANEL_ON=20
PANEL_OFF=25
BEE_ON=30
BEE_OFF=35

stopFlag=False

MOTOR_TRIGGER_A = 16
MOTOR_TRIGGER_B = 20
MOTOR_TRIGGER_C = 21
TRIGER = 5
VARIADOR = 10
BRUSH = 27
LATCH = 22
DIR = 9   # Direction GPIO Pin
STEP = 11  # Step GPIO Pin
CW = 1     # Clockwise Rotation
CCW = 0    # Counterclockwise Rotation
SPR = (360/1.8)*8   # Steps per Revolution (360 / 7.5)
cam_position = 0
delay = .0005

class OpenCVLoop(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

        self.quitFlag = False

        # define the lower and upper boundaries of the "green"
        # ball in the HSV color space, then initialize the
        # list of tracked points
        self.Lower=(0, 0, 0)
        self.Upper=(232, 230, 230)
        self.pts=deque(maxlen=64)

        # if a video path was not supplied, grab the reference
        # to the webcam
        self.camera=cv2.VideoCapture(0)
        
    def run(self):
        global stopFlag

        MAX_FRAMES_AVG=10
        framePtr=0
        framesWithEnemy=[0]*MAX_FRAMES_AVG

        while self.quitFlag == False:
            # grab the current frame
            (grabbed, frame)=self.camera.read()

            # resize the frame, blur it, and convert it to the HSV
            # color space
            frame=imutils.resize(frame, width=640)
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            mask = cv2.inRange(blurred, self.Lower, self.Upper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            mask = 255-mask

            # find contours in the mask and initialize the current
            # (x, y) center of the ball
            cnts=cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)[-2]
            #center=None

            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c=max(cnts, key=cv2.contourArea)
                epsilon = 0.1*cv2.arcLength(c,True)
                approx = cv2.approxPolyDP(c,epsilon,True)
                ((x, y), radius) = cv2.minEnclosingCircle(approx)
                if radius>80:
                    framesWithEnemy[framePtr]=1
                else:
                    framesWithEnemy[framePtr]=0
            else:
                framesWithEnemy[framePtr]=0

            enemyTotal=0
            #print(framesWithEnemy)
            for j in range(0,MAX_FRAMES_AVG):
                enemyTotal=enemyTotal + framesWithEnemy[j]
            if enemyTotal > MAX_FRAMES_AVG/2:
                #print("STOP!!")
                stopFlag = True
            else:
                #print("OK")
                stopFlag = False

            framePtr=framePtr + 1
            if framePtr==MAX_FRAMES_AVG-1:
                framePtr=0
        
            if self.quitFlag:
                break

def get_motors():
    try:
        ser0=serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)
    except:
        ser0=None

    try:
        ser1=serial.Serial('/dev/ttyUSB1', 115200, timeout=0.5)
    except:
        ser1=None
        
    try:
        ser2=serial.Serial('/dev/ttyUSB2', 115200, timeout=0.5)
    except:
        ser2=None

    ports=[ser0, ser1, ser2]

    motorA=None
    motorB=None
    motorC=None

    for port in ports:
        if port is not None:
            if getDriverID(port)==101:
                motorA=port
            elif getDriverID(port)==202:
                motorB=port
            elif getDriverID(port)==303:
                motorC=port
                
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
       msg_cmd=int.from_bytes(ser.read(1), byteorder='little', signed=False)
       msg_type=int.from_bytes(ser.read(1), byteorder='little', signed=False)
       msg_value=int.from_bytes(ser.read(4), byteorder='little', signed=True)
          
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

def stop(ser):
    writeSerialData(ser, STOP, REQUEST, 0)
    return readSerialData(ser)

def restart(ser):
    writeSerialData(ser, RESTART, REQUEST, 0)
    return readSerialData(ser)

def read_coordinates():
    coordinates=[]
    with open("coordinates.txt") as fp:
        for line in fp:
            coordinates.append([int(x) for x in line.split()])
    return coordinates    

def robot_is_moving(A,B,C):
    movA=getState(A)==ROBOT_MOVING
    movB=getState(B)==ROBOT_MOVING
    movC=getState(C)==ROBOT_MOVING
    time.sleep(0.5)
    return movA or movB or movC

def movecamera(angle):
    step_count = int(SPR)
    step_count = (step_count*angle)/360
    if cam_position>angle:
        GPIO.output(DIR, CW)
    if cam_position<angle:
        GPIO.output(DIR, CCW)
    if cam_position==angle:
        break
    for x in range(step_count):
        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)
    cam_position = angle

def get_mov(coordinate):
    X=coordinate[0]
    Y=coordinate[1]

    distance=math.sqrt(X*X+Y*Y)
    if Y==0:
        if X > 0:
            angle=0 #up
        else:
            angle=math.pi #down
    elif X==0:
        if Y > 0:
            angle=3*math.pi/2 #right
        else:
            angle=math.pi/2 #left
    else:
        angle=math.atan(Y/X)
    if X==0 and Y==0:
        angle = 0
        distance = 0 
    print('distance',distance)
    angle_print = math.degrees(angle)-270
    if angle_print<0:
        angle_print = 360+angle_print
    print('angle',angle_print)

    #movecamera(angle_print)

    speedC=int(round(distance*math.sin((math.pi/2)-angle)))
    speedA=int(round(distance*math.sin((7*math.pi/6)-angle)))
    speedB=int(round(distance*math.sin((11*math.pi/6)-angle)))
    speedB=speedB*(-1)
    
    if X==0 and Y!=0:
        speedB=-speedB
        speedA=-speedA

    if Y==0 and X!=0:
        speedB=speedB
        speedA=speedA
        speedC=speedC

    if X<0 and Y<0:
        speedB=-speedB
        speedA=-speedA
        speedC=-speedC

    if X<0 and Y==0:
        speedB=speedB
        speedA=speedA
        speedC=speedC

    if X<0 and Y>0:
        speedB=-speedB
        speedA=-speedA
        speedC=-speedC

    print('SpeedA: ',speedA)
    print('SpeedB: ',speedB)
    print('SpeedC: ',speedC)

    return speedA, speedB, speedC

def get_rotation(coordinate):
    angle=coordinate[2] 

    robot_perimeter=2*math.pi*167

    rotation_distance=int(-robot_perimeter*angle/360)
    print('ROTATION: ',rotation_distance)
    return rotation_distance

def triggerHigh():
    GPIO.output(16,GPIO.HIGH)
    GPIO.output(20,GPIO.HIGH)
    GPIO.output(21,GPIO.HIGH)
    time.sleep(0.5)

def triggerLow():
    GPIO.output(16,GPIO.LOW)
    GPIO.output(20,GPIO.LOW)
    GPIO.output(21,GPIO.LOW)
    time.sleep(0.5)

def ini_pins():
    GPIO.setwarnings(False)        
    GPIO.setmode (GPIO.BCM)

    GPIO.setup(MOTOR_TRIGGER_A, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(MOTOR_TRIGGER_B, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(MOTOR_TRIGGER_C, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(BRUSH,GPIO.OUT)             # initialize GPIO19 as an output
    GPIO.setup(LATCH,GPIO.OUT)
    GPIO.setup(VARIADOR,GPIO.OUT)
    GPIO.setup(TRIGER,GPIO.IN)

    GPIO.setup(DIR, GPIO.OUT)
    GPIO.setup(STEP, GPIO.OUT)

    brush_servo = GPIO.PWM(BRUSH,50)              # GPWM output, with 50Hz frequency
    brush_servo.start(7.5)                   # generate PWM signal with 7.5% duty cycle
    latch_servo = GPIO.PWM(LATCH,50)              
    latch_servo.start(7.5)
    time.sleep(0.5)
    brush_servo.stop()
    latch_servo.stop()

    return brush_servo, latch_servo

def WaitTrigger():
    while GPIO.input(TRIGER)==0:
        continue

def get_action(coordinate):
    action=coordinate[3]

    return action

def control_launcher(run_time,brush,latch):
    esc = GPIO.PWM(VARIADOR,50)              
    esc.start(7.5)
    esc.ChangeDutyCycle(5)
    time.sleep(1)

    for a in range (12,2):
        esc.ChangeDutyCycle(a)
        time.sleep(0.01)
    time.sleep(1)
    for a in range (2,12):
        esc.ChangeDutyCycle(a)
        time.sleep(0.01)
    time.sleep(1)
    for a in range (12,5):
        esc.ChangeDutyCycle(a)
        time.sleep(0.01)
    time.sleep(0.2)
    for a in range (50,59):
        esc.ChangeDutyCycle(a/10)
        time.sleep(0.01)

    t_end = time.time() + run_time #seconds

    while time.time() < t_end:          
        #esc.ChangeDutyCycle(8.5)                                         
        latch.ChangeDutyCycle(8.5)                        #110º
        brush.ChangeDutyCycle(12.5)                 #180º
        time.sleep(1)                                     
        brush.ChangeDutyCycle(2.5)                  # 0º
        time.sleep(1)                                     
        latch.ChangeDutyCycle(12.5)                       #180º
        time.sleep(0.17)

    brush.ChangeDutyCycle(12.5)                 #180º
    latch.ChangeDutyCycle(8.5)                        #110º
    for a in range (59,55):
        esc.ChangeDutyCycle(a/10)
        time.sleep(0.01)
    brush.stop()
    latch.stop()

def main():
    enemyDetectionLoop=OpenCVLoop()
    enemyDetectionLoop.start()

    brush, latch = ini_pins()
    
    WaitTrigger()
    # State 0: activate motors
    motorA, motorB, motorC=get_motors()

    # State 1: read coordinates list
    coordinates=read_coordinates()
    
    # For each coordinate
    for c in coordinates:
        print('COORDINATES: ',c)
        speedA, speedB, speedC=get_mov(c)

        setDistance(motorA, speedA*3)
        setDistance(motorB, speedB*3)
        setDistance(motorC, speedC*3)
        setSpeedSetpoint(motorA,speedA)
        setSpeedSetpoint(motorB,speedB)
        setSpeedSetpoint(motorC,speedC)
        triggerHigh()

        # Wait for the robot to stop
        while robot_is_moving(motorA, motorB, motorC):
            if stopFlag == True:
                print('ROBOT IS BLOCKED')
                triggerLow()
            else:
                print('ROBOT IS MOVING')
                triggerHigh()
            time.sleep(0.2)
        triggerLow()
        
        rotation=get_rotation(c)
        setDistance(motorA, rotation)
        setDistance(motorB, -rotation)
        setDistance(motorC, rotation)
        setSpeedSetpoint(motorA,300)
        setSpeedSetpoint(motorB,300)
        setSpeedSetpoint(motorC,-300)
        triggerHigh()

        # Wait for the robot to stop
        while robot_is_moving(motorA, motorB, motorC):
            if stopFlag == True:
                print('ROBOT IS BLOCKED')
                triggerLow()
            else:
                print('ROBOT IS MOVING')
                triggerHigh()
            time.sleep(0.2)
        triggerLow()

        action=get_action(c)
        if action==LAUNCH_BALLS:
            control_launcher(6,brush,latch)
            
        print ('END OF COORDINATES')
    print('FOOBAR')
    enemyDetectionLoop.quitFlag = True
            
if __name__=="__main__":
    main()
