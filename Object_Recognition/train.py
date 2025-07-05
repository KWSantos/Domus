from ultralytics import YOLO

model = YOLO("yolov8m.pt")

results = model.train(data="Domestic-Objetcs.v1i.yolov8/data.yaml", epochs=100, imgsz=320)