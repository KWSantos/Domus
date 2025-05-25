from ultralytics import YOLO

model = YOLO("yolov8n.pt")

results = model.train(data="Home_Person_Dataset.v1i.yolov8/data.yaml", epochs=100, imgsz=320)