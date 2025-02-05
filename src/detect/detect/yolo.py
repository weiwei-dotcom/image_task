import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node
from ultralytics import YOLO
from interface.srv import TarState
from rclpy.parameter import Parameter
import cv2
import yaml
import random


class YoloDetector(Node):
    
    def __init__(self):
        super().__init__('yolo_detect')
        self.InitParams()
        self.bridge = CvBridge()
        self.model = YOLO(self.config["model_path"])
        self.img_sub = self.create_subscription(Image, 'image', self.ImgCallBack, 10)
        self.get_tar_server = self.create_service(TarState, 'get_tar_state', self.GetTarStateCallback)
        self.color = [[]]
        for i in range(1000):
            self.color.append([random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)])
        
    def GetTarStateCallback(self, request, response):
        response.success = False
        max_area = 0.0
        for result in self.results:
            for box in result.boxes:
                if request.name is not result.names[int(box.cls[0])]:
                    break
                temp_area = box.xywhn[0][2] * box.xywhn[0][3]
                if temp_area < self.config["min_area"]:
                    break
                response.success = True
                if max_area < temp_area:
                    max_area = temp_area
                    response.x = box.xywh[0][0]
                    response.y = box.xywh[0][1]
        
    def InitParams(self):
        with open("/home/wl/Documents/image_task/src/detect/config/yolo_detect.yaml", mode="r", encoding="utf8") as file:
            self.config_yaml = yaml.load(file, Loader=yaml.FullLoader)
        self.config = {}
        self.declare_parameter("model_path", "/home/ww")
        self.declare_parameter("fps", 30)
        self.declare_parameter("confidence", 0.5)
        self.declare_parameter("font_size", 2)
        self.declare_parameter("thickness", 3)
        self.declare_parameter("min_area", 0.1)
        self.set_parameters([
            Parameter('model_path', Parameter.Type.STRING, self.config_yaml["yolo_detect"]["ros__parameters"]["model_path"]),
            Parameter('fps', Parameter.Type.INTEGER, self.config_yaml["yolo_detect"]["ros__parameters"]["fps"]),
            Parameter('confidence', Parameter.Type.DOUBLE, self.config_yaml["yolo_detect"]["ros__parameters"]["confidence"]),
            Parameter('font_size', Parameter.Type.INTEGER, self.config_yaml["yolo_detect"]["ros__parameters"]["font_size"]),
            Parameter('min_area', Parameter.Type.DOUBLE, self.config_yaml["yolo_detect"]["ros__parameters"]["min_area"]),
            Parameter('thickness', Parameter.Type.INTEGER, self.config_yaml["yolo_detect"]["ros__parameters"]["thickness"])])
        self.config["model_path"] = self.get_parameter("model_path").get_parameter_value().string_value
        self.config["fps"] = self.get_parameter("fps").get_parameter_value().integer_value
        self.config["confidence"] = self.get_parameter("confidence").get_parameter_value().double_value
        self.config["min_area"] = self.get_parameter("min_area").get_parameter_value().double_value
        self.config["font_size"] = self.get_parameter("font_size").get_parameter_value().integer_value
        self.config["thickness"] = self.get_parameter("thickness").get_parameter_value().integer_value
        
        
    def ImgCallBack(self, img_msg):
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        self.results = self.model.predict(cv_img, conf=self.config["confidence"])
        for result in self.results:
            for box in result.boxes:
                cv2.rectangle(cv_img, (int(box.xyxy[0][0]), int(box.xyxy[0][1])), (int(box.xyxy[0][2]), int(box.xyxy[0][3])), self.color[int(box.cls[0])], self.config["thickness"])
                cv2.putText(cv_img, f"{result.names[int(box.cls[0])]}", (int(box.xyxy[0][0]), int(box.xyxy[0][1]+25)), cv2.FONT_HERSHEY_PLAIN, self.config["font_size"], self.color[int(box.cls[0])], self.config["thickness"])
                cv2.putText(cv_img, f"{int(box.xywh[0][0])}, {int(box.xywh[0][1])}", (int(box.xyxy[0][0]), int(box.xyxy[0][3]-7)), cv2.FONT_HERSHEY_PLAIN, self.config["font_size"], self.color[int(box.cls[0])], self.config["thickness"])
        cv2.imshow("yolo-detect-result", cv_img)
        cv2.waitKey(int(1000/self.config["fps"]+1))

def main(args = None):
    rclpy.init(args = args)
    node = YoloDetector()
    rclpy.spin(node)
    rclpy.shutdown()
