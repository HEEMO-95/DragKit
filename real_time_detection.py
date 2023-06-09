import cv2
import numpy as np
from ultralytics import YOLO

path = (
    "C:/Users/ksa_j/Documents/Codes/VS/python/YOLO/runs/detect/train/weights/best.onnx"
)
image = "C:/Users/ksa_j/Documents/Codes/VS/python/YOLO/test1.png"
cap = cv2.VideoCapture("F1.mp4")
model = YOLO(path)

while cap.isOpened():
    ret, frame = cap.read()
    frame = cv2.resize(frame, dsize=(1024, 1024), interpolation=cv2.INTER_LINEAR)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    if not ret:
        break

    results = model.predict(source=frame, stream=False, show=True)
    # print(results[0])

    for result in results:
        boxes = result.boxes  # Boxes object for bbox outputs
        masks = result.masks  # Masks object for segmentation masks outputs
        probs = result.probs  # Class probabilities for classification outputs

    if len(boxes) != 0:
        # print(boxes.data.tolist())
        x1, y1, x2, y2 = boxes.data.tolist()[0][:4]
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        cropped_box = frame[y1:y2, x1:x2]
        cropped_box = cv2.resize(
            cropped_box, dsize=(640, 640), interpolation=cv2.INTER_LINEAR
        )
        cv2.imshow("result", cropped_box)
        # Print the coordinates

cap.release()
cv2.destroyAllWindows()
