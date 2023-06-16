import subprocess
import os
import cv2
from AI import *
import time

path = "/home/jetson/Desktop/Dragkit/DRAG_Shape_detector/runs/detect/train/weights/best.pt"
model = Detector(path)
print('model loaded')
vid = cv2.VideoCapture('/home/jetson/Desktop/Dragkit/F1.mp4')
print('start capturing')
objs = []  # Reuse the same list for object detections
while vid.isOpened():
    ret, frame = vid.read()
    s = time.time()
    objs.clear()  # Clear the list for new object detections
    objs = model.predict(frame, stream=True)
    for obj in objs:
        print('-----------------------------------------')
        print(obj[0], ',', obj[2], ',', obj[3], ',', obj[4])
        color = color_detection(obj[1])
        print(color)
        charc = alphabetic_detection(obj[1])
        print(charc)
        print('-----------------------------------------')
    # cv2.imshow('frame', frame)
    e = time.time()
    print('time elapsed: ', (e - s))

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# After the loop, release the capture object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()

