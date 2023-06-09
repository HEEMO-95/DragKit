import cv2
import numpy as np
from ultralytics import YOLO
from AI import *

#path = (
#    "C:/Users/ksa_j/Documents/Codes/VS/python/YOLO/runs/detect/train/weights/best.onnx"
#)
image = "E:/PIC/Sequence 011469.png"
#cap = cv2.VideoCapture("F1.mp4")
#model = YOLO(path)


#ret, frame = cap.read()
frame = cv2.imread(image)
frame = cv2.resize(frame, dsize=(1024, 1024), interpolation=cv2.INTER_CUBIC)
#frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)


obj,prob = object_detection(frame)
# print(results[0])
print(obj.xy)
print(obj.alphabet)
print(obj.shape)
print(obj.color)
print(prob)
#boxes = results.boxes  # Boxes object for bbox outputs
#masks = results.masks  # Masks object for segmentation masks outputs
#probs = results.probs  # Class probabilities for classification outputs
# print(boxes.data.tolist())
cv2.imshow("result", frame)
# Print the coordinates
cv2.waitKey(0)