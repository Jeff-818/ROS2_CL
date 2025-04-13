import torch
import cv2
import numpy as np
from ultralytics import YOLO
from torch2trt import TRTModule
from torch2trt import torch2trt
# from models.yolo import Model  # 你需要有模型結構
import os

# class YoloToTrt:
#     model = torch.load("yolov10s.pt")
#     model = model.to("cuda").eval()

#     # 建立 dummy input
#     x = torch.ones((1, 3, 640, 640)).to("cuda")

#     # 轉換成 TensorRT
#     model_trt = torch2trt(model, [x])

#     # 儲存 TensorRT model
#     torch.save(model_trt.state_dict(), "yolov10s_trt.pth")


# class YOLODetector:
#     def __init__(self):
#         self.device = torch.device("cuda")
        
#         # 設定你的 TensorRT 模型的 .pth 路徑
#         model_path = "/home/jetson/Workspace/ros2_cl/src/car_camera/mode/yolov10s_trt.pth"
        
#         # 初始化 TensorRT 模型      我要如何取得yolo的模型結構
#         self.model = TRTModule()
#         self.model.load_state_dict(torch.load(model_path))
#         self.model.eval()

#     def detect_objects(self, image):
#         # 前處理：resize -> normalize -> to tensor
#         img = cv2.resize(image, (640, 640))
#         img = img / 255.0
#         img = torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0).float().to(self.device)

#         # 推論
#         with torch.no_grad():
#             detections = self.model(img)

#         return detections



import torch
import os
from ultralytics import YOLO
from models.yolo import Model  # 確保你有 yolov10 的原始碼並在 PYTHONPATH 裡

class YOLODetector:
    def __init__(self):
        model_path = "/home/jetson/Workspace/ros2_cl/src/car_camera/mode/yolov10s/yolov10s.pt"
        config_path = "/home/jetson/Workspace/ros2_cl/src/car_camera/mode/yolov10s/yolov10s.yaml"  # 模型定義
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.model = Model(cfg=config_path, ch=3, nc=80).to(self.device)  # ch=3 for RGB, nc=80 for COCO
        state_dict = torch.load(model_path, map_location=self.device)
        self.model.load_state_dict(state_dict)  # 載入訓練好的參數
        self.model.eval()
