import rclpy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class ImageSubscriber:
    def __init__(self, node):
        self.node = node
        self.bridge = CvBridge()

        # 訂閱影像 & 相機內參
        self.color_sub = message_filters.Subscriber(node, Image, "/zed2i/zed_node/rgb/image_rect_color")
        self.depth_sub = message_filters.Subscriber(node, Image, "/zed2i/zed_node/depth/depth_registered")
        self.color_info_sub = message_filters.Subscriber(node, CameraInfo, "/zed2i/zed_node/rgb/camera_info")
        self.depth_info_sub = message_filters.Subscriber(node, CameraInfo, "/zed2i/zed_node/depth/camera_info")


        # 使用 ApproximateTimeSynchronizer 來同步影像
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.color_info_sub, self.depth_info_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)

    def sync_callback(self, color_msg, depth_msg, color_info_msg, depth_info_msg):
        """ 當 RGB、Depth 影像同步時，這個 callback 會被觸發 """
        color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")

        # 傳給主節點進行 YOLO 偵測
        self.node.process_synchronized_images(color_image, depth_image, color_info_msg, depth_info_msg)


        
        