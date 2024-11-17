import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node
from ultralytics import YOLOv10


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.bridge = CvBridge()
        self.model = YOLOv10("/home/ww/Documents/yolov10/yolov10m.pt")
        self.img_sub = self.create_subscription(
            Image,
            'image',
            self.ImgCallBack,
            10)
        
    def ImgCallBack(self, img_msg):
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        results = self.model(cv_img)
        boxes = results.