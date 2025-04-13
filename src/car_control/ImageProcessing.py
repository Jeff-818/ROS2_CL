import cv2
import sys
import numpy as np
import rclpy
import time
import torch
import traceback
import threading

from mode.image_subscring import ImageSubscriber
from mode.yolo_deterctor import YOLODetector
from mode.detections_pointcloud import  DepthDetections

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32, Float64MultiArray
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import Path


from queue import Queue,Empty
from nav_msgs.msg import Odometry
import math
# sys.path.append(os.path.dirname(os.path.abspath(__file__)))

#------------------------------#
#   threading 1 : sub/imag -> cv -> queue
#   threading 2 : queue -> 偵測 ->座標 -> 座標轉換 -> 畫框 -> marker -> pub
#   threading 3 : 
#"rgb/image_rect_color"
# "rgb/camera_info"
# "depth/depth_registered"
# "depth/camera_info"
# "point_cloud/cloud_registered"
# "pose_with_covariance"
# "imu/data"

# "map"
# "tf"
# "辨識物座標"
# "cmd_vel"  # Nav2 發送給 ESP32 的移動指令
# "odom"  # ESP32 回傳 encoder 位置
# "local_costmap"  # RTAB-Map 生成的局部導航資訊
#------------------------------#


class ImageProcessing (Node):
    def __init__(self):
        super().__init__('ImageProcessing')
        
        self.point_cloud_cloud_registered = self.create_subscription(Int32, 'point_cloud/cloud_registered', 10)
        self.pose_with_covariance = self.create_subscription(Int32, 'pose_with_covariance', self.pose_with_covariance_callback, 10)
        self.imu_data = self.create_subscription(Int32, 'imu/data', 10)
        self.imu_data = self.create_subscription(Int32, 'odom', 10)
        self.imu_data = self.create_subscription(Int32, 'oobjects', 10)
        self.imu_data = self.create_subscription(Int32, 'object_markers', 10)
        self.imu_data = self.create_subscription(Int32, '/diagnostics', 10)
        
        self.image_subscriber = ImageSubscriber(self)
        self.yolo_detector = YOLODetector()
        
        self.stop_processing = threading.Event()    #//
        self.inference_queue = Queue(maxsize=5)
    
    
    def process_synchronized_images(self, color_image, depth_image, info_msg):
        """
            Thread 1:
                不斷從 image_subscriber 拿資料(color/depth) -> 基礎處理 -> 放入 inference_queue
        """
        self.get_logger().info("decode_thread_func started")
        while not self.stop_processing.is_set():
            data = self.image_subscriber.get_data(timeout=1)
            if data is None:
                continue
            try:
                color_msg, depth_msg = data
                if not self.inference_queue.full():
                    self.inference_queue.put((cv_color, cv_depth))
                else:
                    # self.get_logger().warn("inference_queue is full, dropping frame.")
                    pass

            except Exception as e:
                self.get_logger().error(f"decode_thread_func error: {e}")
                self.get_logger().error(traceback.format_exc())
                
        self.inference_queue.put((color_image, depth_image))




    def yolo_thread_func(self):
        """
        Thread 2:
          不斷從 inference_queue 拿 color 影像, 做 YOLO 偵測, 之後將結果整合後續(如pccallback)
        """
        self.get_logger().info("yolo_thread_func started")
        while not self.stop_processing.is_set():
            try:
                cv_color, cv_depth = self.inference_queue.get(timeout=1)
            except Empty:
                continue
            try:
                # YOLO 偵測
                detections = self.yolo_detector.detect_objects(cv_color)
                if len(detections) == 0:
                    self.egg_navigator.yolo_received = False
                    self.yolo_received = False
                else:
                    self.egg_navigator.yolo_received = True
                    self.yolo_received = True

                self.yolov8_item = detections

                if self.depth_processor and len(detections) > 0:
                    self.detections_pointcloud(cv_depth, detections)
                # 發布影像
                self.detections_rectangle(cv_color, detections)
            except Exception as e:
                self.get_logger().error(f"yolo_thread_func error: {e}")
                self.get_logger().error(traceback.format_exc())
    
    
    
    
