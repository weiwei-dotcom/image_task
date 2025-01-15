from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    normal_camera_pkg_dir = get_package_share_directory("camera")
    normal_camera_file_name = "normal_camera.yaml"
    normal_camera_file_path = os.path.join(normal_camera_pkg_dir, "config", normal_camera_file_name)

    yolo_pkg_dir = get_package_share_directory("detect")
    yolo_file_name = "yolo_detect.yaml"
    yolo_file_path = os.path.join(yolo_pkg_dir, "config", yolo_file_name)
    
    normal_camera_node = Node(
        package="camera",
        executable='normal',
        parameters = [normal_camera_file_path]
    )
    yolo_detect_node = Node(
        package="detect",
        executable='yolo',
        parameters = [yolo_file_path]
    )
    launch_description = LaunchDescription(
        [normal_camera_node, yolo_detect_node]
    )
    return launch_description