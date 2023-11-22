# based on https://pjreddie.com/darknet/yolo/
# and a promt by chatgpt
# https://pyserial.readthedocs.io/en/latest/shortintro.html

from ultralytics import YOLO
import numpy as np
import cv2
import serial
import json
from map import map_value

# set up serial communication with arduino board
ser = serial.Serial('/dev/cu.usbmodem1101', 9600)

# constant for openCV
CONFIDENCE_THRESHOLD = 0.85
GREEN = (0, 255, 0)

# setup variable for communication
buzz = False

# load the model yolov8
model = YOLO("yolov8n.pt")

# setup openCV video capture
cv2.startWindowThread()
cap = cv2.VideoCapture(0)

# loop for ever
while(True):
    buzz = False
    # read the one frame of the video
    ret, frame = cap.read()

    # get height and width
    height, width, _ = frame.shape

    # get the detections
    detections = model(frame)[0]

    # convert detection to a list
    det = detections.boxes.data.tolist()

    # list to store the detected objects
    to_follow = []

    # traverse the detections
    for data in det:
        # extract the confidence (i.e., probability) associated with the detection
        confidence = data[4]
        class_id = data[5] # class id to, if the detection is a human, a fruit or what ever

        # filter out weak detections by ensuring the 
        # confidence is greater than the minimum confidence
        # and only detect humans
        if float(confidence) < CONFIDENCE_THRESHOLD and class_id != 0:
            continue # go to the next item

        # if the confidence is greater than the minimum confidence,
        # draw the bounding box on the frame
        xmin, ymin, xmax, ymax = int(data[0]), int(data[1]), int(data[2]), int(data[3])
        cv2.rectangle(frame, (xmin, ymin) , (xmax, ymax), GREEN, 2)
        to_follow.append((xmin, xmax))
        if len(to_follow) >= 3:
            break
    
    # if there is only one person detected
    if len(to_follow) == 1:
        a = to_follow[0]
        x = (a[0] + a[1])/2 # calculate the horizontal position of the detection
        angle1 = map_value(x, 0, width, 155, 25) # map the position to an angle between 155 and 25
        if x < width/3 or x > 2*width/3: # detection is in the middle of the area
            buzz = True

        m = str.encode(json.dumps({'servoL': angle1, 'servoC': angle1, 'servoR': angle1, 'buzz': buzz}))
        ser.write(m)
    elif len(to_follow) > 1: # there is more than one detection
        angleL = 90
        angleC = 90
        angleR = 90
        for obj in to_follow:
            x = (obj[0] + obj[1])/2
            if x < width /3:
                angleL = map_value(x, 0, width/3, 155, 25)
            elif x < 2*width/3:
                angleC = map_value(x, width/3, 2*width/3, 155, 25)
            else:
                angleR = map_value(x, 2*width/3, width, 155, 25)
        
        message = str.encode(json.dumps({'servoL': angleL, 'servoC': angleC, 'servoR': angleR, 'buzz': False}))
        print(message)
        ser.write(message)
    else: # look to the front
        m = str.encode(json.dumps({'servoL': 90, 'servoC': 90, 'servoR': 90, 'buzz': buzz}))
        ser.write(m)

    cv2.imshow("Frame", frame) # display frame

    ser.flush();

    if cv2.waitKey(1) & 0xFF == ord('q'): # exit if you press letter q on the keyboard
        break

# close everything
cap.release()
cv2.destroyAllWindows()
cv2.waitKey(1)
ser.close()
