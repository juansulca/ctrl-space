# based on https://pjreddie.com/darknet/yolo/
# and a promt by chatgpt
# https://pyserial.readthedocs.io/en/latest/shortintro.html

from ultralytics import YOLO
import numpy as np
import cv2
import serial
import json
from map import map_value

ser = serial.Serial('/dev/cu.usbmodem21101', 9600)

CONFIDENCE_THRESHOLD = 0.8
GREEN = (0, 255, 0)

control = {'servoVal': 0, 'servo2': 0, 'servo3': 0, 'buzz': False}

# net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
# net = cv2.dnn.readNet("yolov3-tiny.weights", "yolov3-tiny.cfg")
# layer_names = net.getUnconnectedOutLayersNames()

model = YOLO("yolov8n.pt")

cv2.startWindowThread()
cap = cv2.VideoCapture(0)

while(True):
    ret, frame = cap.read()

    # frame = cv2.resize(frame, (640, 480))

    height, width, _ = frame.shape

    detections = model(frame)[0]

    det = detections.boxes.data.tolist()

    to_follow = [];

    for data in det:
        # extract the confidence (i.e., probability) associated with the detection
        confidence = data[4]
        class_id = data[5]

        # filter out weak detections by ensuring the 
        # confidence is greater than the minimum confidence
        if float(confidence) < CONFIDENCE_THRESHOLD and class_id != 0:
            continue

        # if the confidence is greater than the minimum confidence,
        # draw the bounding box on the frame
        xmin, ymin, xmax, ymax = int(data[0]), int(data[1]), int(data[2]), int(data[3])
        cv2.rectangle(frame, (xmin, ymin) , (xmax, ymax), GREEN, 2)
        to_follow.append((xmin, xmax))
    
    if len(to_follow) > 0:
        # for obj in to_follow:
        #     x = (obj[0] + obj[1])/2
        #     # angle = map_value(x, 0, width, 155, 25)
        #     if x < width /3:
        #         angle = map_value(x, 0, width/3, 155, 25)
        #         control['servoVal'] = angle
        #     elif x < 2*width/3:
        #         angle = map_value(x, width/3, 2*width/3, 155, 25)
        #         control['servo2'] = angle
        #     else:
        #         angle = map_value(x, 2*width/3, width, 155, 25)
        #         control['servo3'] = angle
        a = to_follow[0]
        x = (a[0] + a[1])/2
        angle1 = map_value(x, 0, width, 155, 25)
        # angle2 = map_value(x, 0, width, 155, 25)
        # angle3 = map_value(x, 0, width, 155, 25)
        # m = str.encode(json.dumps(control))
        m = str.encode(json.dumps({'servoVal': angle1, 'servo2': angle1, 'servo3': angle1}))
        ser.write(m)

    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
cv2.waitKey(1)
