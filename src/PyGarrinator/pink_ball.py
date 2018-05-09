from collections import deque
import numpy as np
import argparse
import imutils
import cv2

Upper = (232, 230, 230)
Lower = (0, 0, 0)
pts=deque(maxlen=64)

camera = cv2.VideoCapture(0)

while True:
    # grab the current frame
    (grabbed, orig)=camera.read()
    frame = imutils.resize(orig, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    '''
    lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
    lab_planes = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=80, tileGridSize=(6,6))
    
    lab_planes[0] = clahe.apply(lab_planes[0])

    lab = cv2.merge(lab_planes)
    bgr = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    '''
    #hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(blurred, Lower, Upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    mask = 255-mask
    
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        epsilon = 0.1*cv2.arcLength(c,True)
        approx = cv2.approxPolyDP(c,epsilon,True)
        ((x, y), radius) = cv2.minEnclosingCircle(approx)
        
        # only proceed if the radius meets a minimum size
        if radius > 80:
            print(radius)
        else:
            print('not')

    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    cv2.imshow("mask", mask)
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
            break

# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()

