from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    normal_camera_node = Node(
        package="camera",
        executable='normal',
        parameters = ["/home/wl/Documents/image_task/src/camera/config/normal_camera.yaml"]
    )
    yolo_detect_node = Node(
        package="detect",
        executable='yolo',
        parameters = ["/home/wl/Documents/image_task/src/detect/config/yolo_detect.yaml"]
    )
    launch_description = LaunchDescription(
        [normal_camera_node, yolo_detect_node]
    )
    return launch_description