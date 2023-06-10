from DRAG_shape_Detector import Shape_Model
import cv2

path = "./runs/detect/train/weights/best_dynamic.onnx"
model1 = Shape_Model(path)
cap = cv2.VideoCapture("Video_to_test.mp4")


while cap.isOpened():
    ret, frame = cap.read()
    result = model1.predict(frame, show=True)
    for i in result:
        print("Shape: ", i[0])
        crroped = cv2.resize(i[1], dsize=(640, 640))

        # Define the text, position, font, and scale for the text overlay
        text = "Score: " + "{:.2f}%".format(i[2] * 100)
        position = (0, 600)  # Specify the top-left corner of the text
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.8
        color = (255, 0, 0)  # White color
        cv2.putText(crroped, text, position, font, scale, color, thickness=2)
        cv2.imshow(i[0], crroped)
        print("Score: ", i[2])
        print("X1: ", i[3])
        print("X2: ", i[4])
        print("Y1: ", i[5])
        print("Y2: ", i[6])
