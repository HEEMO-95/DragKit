from ultralytics import YOLO

path = "runs/detect/train/weights/best_dynamic.onnx"
model = YOLO(path)
model.export(format="onnx", dynamic=True)
