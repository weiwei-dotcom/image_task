import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node
from ultralytics import YOLO
import cv2
import yaml
import random


class YoloDetector(Node):
    
    def __init__(self):
        super().__init__('yolo_detector')
        self.InitParams()
        self.bridge = CvBridge()
        self.model = YOLO(self.config["model_path"])
        self.img_sub = self.create_subscription(
            Image,
            'image',
            self.ImgCallBack,
            10)
        self.color = [[]]
        for i in range(1000):
            self.color.append([random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)])
        
        
    def InitParams(self):
        with open("/home/ww/Documents/image_task/src/detect/config/yolo_detect.yaml", mode="r", encoding="utf8") as file:
            self.config_yaml = yaml.load(file, Loader=yaml.FullLoader)
        self.config = {}
        self.declare_parameter("model_path", "/home/ww")
        self.set_parameters('model_path', self.config_yaml["model_path"])
        self.config["model_path"] = self.get_parameter("model_path").get_parameter_value().string_value
        self.declare_parameter("fps", 30)
        self.set_parameters('fps', self.config_yaml["fps"])
        self.config["fps"] = self.get_parameter("fps").get_parameter_value().integer_value
        self.declare_parameter("confidence", 0.5)
        self.set_parameters('confidence', self.config_yaml["confidence"])
        self.config["confidence"] = self.get_parameter("confidence").get_parameter_value().double_value
        self.declare_parameter("font_size", 2)
        self.set_parameters("font_size", self.config_yaml["font_size"])
        self.config["font_size"] = self.get_parameter("font_size").get_parameter_value().integer_value
        self.declare_parameter("font_style", "Arial")
        self.set_parameters("font_style", self.config_yaml["font_style"])
        self.config["font_style"] = self.get_parameter("font_style").get_parameter_value().string_value
        self.declare_parameter("thickness", 3)
        self.set_parameter("thickness", self.config_yaml["thickness"])
        self.config["thickness"] = self.get_parameter("thickness").get_parameter_value().integer_value
        
        
    def ImgCallBack(self, img_msg):
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        results = self.model.predict(cv_img)
        for result in results:
            for box in result.boxes:
                cv2.rectangle(cv_img, (box.xyxy[0][0], box.xyxy[0][1]), (box.xyxy[0][2],box.xyxy[0][3]), self.color[box.cls], self.config["thickness"])
                cv2.putText()
        boxes = results.boxes