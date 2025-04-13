import sys
sys.path.append("/usr/local/zed/get_python_api.py")
import numpy as np
import cv2
import pyzed.sl as sl
import argparse
import ogl_viewer.viewer as gl
import cv_viewer.tracking_viewer as cv_viewer
import platform
from collections import deque


class DepthDetections ():
    def __init__ (self,):
            # 初始化相機
        zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080  # 固定 1080p
        init_params.camera_fps = 30  # 固定 30 FPS
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.coordinate_units = sl.UNIT.METER
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA 
        init_params.depth_maximum_distance = 10.0  # 設定最大深度 10 公尺

        # 開啟相機
        if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            print("ZED 相機開啟失敗！")
            exit(1)

        # 啟用位置追蹤
        tracking_params = sl.PositionalTrackingParameters()
        zed.enable_positional_tracking(tracking_params)

        # 啟用物件偵測
        obj_param = sl.ObjectDetectionParameters()
        obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_FAST
        obj_param.enable_tracking = True
        zed.enable_object_detection(obj_param)

        # 設定偵測結果
        objects = sl.Objects()
        detection_parameters_rt = sl.ObjectDetectionRuntimeParameters()
        detection_parameters_rt.detection_confidence_threshold = 60  # 最低信心分數 60

        # 影像處理迴圈
        while True:
            if zed.grab() == sl.ERROR_CODE.SUCCESS:
                image = sl.Mat()
                point_cloud = sl.Mat()
                zed.retrieve_image(image, sl.VIEW.LEFT)
                zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
                zed.retrieve_objects(objects, detection_parameters_rt)
                
                frame = image.get_data()
                for obj in objects.object_list:
                    position = obj.position  # 3D 座標 (X, Y, Z)
                    print(f"物件 ID {obj.id}: X={position[0]:.2f}, Y={position[1]:.2f}, Z={position[2]:.2f}")
                    
                    # 獲取物件的2D邊界框
                    bounding_box_2d = obj.bounding_box_2d
                    
                    # 繪製邊界框
                    top_left = (int(bounding_box_2d[0][0]), int(bounding_box_2d[0][1]))
                    bottom_right = (int(bounding_box_2d[2][0]), int(bounding_box_2d[2][1]))
                    
                    # 根據物件類別選擇顏色
                    color = (0, 255, 0)  # 綠色作為預設
                    
                    # 繪製矩形框
                    cv2.rectangle(frame, top_left, bottom_right, color, 2)
                    
                    # 顯示物件ID和類別
                    label = f"ID: {obj.id} ({obj.label})"
                    cv2.putText(frame, label, (top_left[0], top_left[1] - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    
                    # 顯示距離
                    distance = f"Dist: {position[2]:.2f}m"
                    cv2.putText(frame, distance, (top_left[0], top_left[1] + 15), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                            
                cv2.imshow("ZED", frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                

                # 釋放資源
                zed.close()
                cv2.destroyAllWindows()

            還是一樣


