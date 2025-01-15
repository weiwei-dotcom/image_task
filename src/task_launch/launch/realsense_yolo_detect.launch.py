from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    realsense_pkg_dir = get_package_share_directory("camera")
    realsense_file_name = "realsense_camera.yaml"
    realsense_file_path = os.path.join(realsense_pkg_dir, "config", realsense_file_name)

    yolo_pkg_dir = get_package_share_directory("detect")
    yolo_file_name = "yolo_detect.yaml"
    yolo_file_path = os.path.join(yolo_pkg_dir, "config", yolo_file_name)
    
    realsense_camera_node = Node(
        package="camera",
        executable='realsense',
        parameters = [realsense_file_path]
    )
    yolo_detect_node = Node(
        package="detect",
        executable='yolo',
        parameters = [yolo_file_path]
    )
    launch_description = LaunchDescription(
        [realsense_camera_node, yolo_detect_node]
    )
    return launch_description