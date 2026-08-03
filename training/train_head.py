import os
from ultralytics import YOLO

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

model = YOLO("yolov8n.pt")

model.train(
    data=os.path.join(PROJECT_ROOT, "mounted_screw_dataset", "data.yaml"),
    epochs=50,
    imgsz=640,
    batch=16,
    name="head_detect",
)
