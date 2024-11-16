#include "normal_camera.hpp"

void NormalCamera::InitParam() {
    // 读取参数
    _config_yaml = YAML::LoadFile("../config/normal_camera.yaml");

    this->declare_parameter("device_id", 0);
    this->set_parameter(rclcpp::Parameter("device_id", _config_yaml["device_id"]));
    this->get_parameter("device_id", _config.device_id);

    this->declare_parameter("frame_width", 640);
    this->set_parameter(rclcpp::Parameter("frame_width", _config_yaml["frame_width"]));
    this->get_parameter("frame_width", _config.frame_width);

    this->declare_parameter("frame_height", 480);
    this->set_parameter(rclcpp::Parameter("frame_height", _config_yaml["frame_height"]));
    this->get_parameter("frame_height", _config.frame_height);

    this->declare_parameter("fps", 30);
    this->set_parameter(rclcpp::Parameter("fps", _config_yaml["fps"]));
    this->get_parameter("fps", _config.fps);
    // 读取相机内参
    _config.intrinsic = cv::Mat::zeros(3, 3, CV_64F);
    _config.intrinsic.at<double>(2,2) = 0;
    _config.dist_coeff = cv::Mat::zeros(1, 5, CV_64F);

    this->declare_parameter("fx", 1.0);
    this->set_parameter(rclcpp::Parameter("fx", _config_yaml["fx"]));
    this->get_parameter("fx", _config.intrinsic.at<double>(0,0));

    this->declare_parameter("fy", 1.0);
    this->set_parameter(rclcpp::Parameter("fy", _config_yaml["fy"]));
    this->get_parameter("fy", _config.intrinsic.at<double>(1,1));

    this->declare_parameter("cx", 320);
    this->set_parameter(rclcpp::Parameter("cx", _config_yaml["cx"]));
    this->get_parameter("cx", _config.intrinsic.at<double>(0,2));

    this->declare_parameter("cy", 240);
    this->set_parameter(rclcpp::Parameter("cy", _config_yaml["cy"]));
    this->get_parameter("cy", _config.intrinsic.at<double>(1,2));

    this->declare_parameter("k1", 0.0);
    this->set_parameter(rclcpp::Parameter("k1", _config_yaml["k1"]));
    this->get_parameter("k1", _config.dist_coeff.at<double>(0,1));

    this->declare_parameter("k2", 0.0);
    this->set_parameter(rclcpp::Parameter("k2", _config_yaml["k2"]));
    this->get_parameter("k2", _config.dist_coeff.at<double>(0,1));

    this->declare_parameter("p1", 0.0);
    this->set_parameter(rclcpp::Parameter("p1", _config_yaml["p1"]));
    this->get_parameter("p1", _config.dist_coeff.at<double>(0,2));

    this->declare_parameter("p2", 0.0);
    this->set_parameter(rclcpp::Parameter("p2", _config_yaml["p2"]));
    this->get_parameter("p2", _config.dist_coeff.at<double>(0,3));

    this->declare_parameter("k3", 0.0);
    this->set_parameter(rclcpp::Parameter("k3", _config_yaml["k3"]));
    this->get_parameter("k3", _config.dist_coeff.at<double>(0,4));
}

NormalCamera::NormalCamera():rclcpp::Node("normal_camera") {

    // 初始化读取参数
    InitParam();

    // 打开摄像头
    OpenCamera();
    _cv2msg.encoding = sensor_msgs::image_encodings::BGR8;
    // 创建发布者
    this->_iamge_pub = this->create_publisher<sensor_msgs::msg::Image>("iamge", 10);
    // 创建发布图像消息定时器
    this->_pub_timer = this->create_wall_timer(
        std::chrono::milliseconds(int(1000/_config.fps)),
        std::bind(&NormalCamera::PubImgCallback, this)
    );
    // 将摄像头捕捉到的图像进行发布
}

void NormalCamera::OpenCamera() {
    _cap.open(_config.device_id, cv::CAP_V4L2);
    int temp_device_id = _config.device_id;
    if (!_cap.isOpened()) {
        RCLCPP_WARN(this->get_logger(), "Camera open with device ID %d failed", _config.device_id);
        if (_config.device_id == 0) {
            temp_device_id = 1;
        } else temp_device_id = _config.device_id - 1;
        RCLCPP_INFO(this->get_logger(), "Trying to open with device ID %d ", temp_device_id);
        _cap.open(temp_device_id, cv::CAP_V4L2);
        if (!_cap.isOpened()) {
            RCLCPP_WARN(this->get_logger(), "Camera open with device ID %d failed", temp_device_id);
            if (_config.device_id == 0) {
                temp_device_id = 2;
            } else temp_device_id = _config.device_id + 1;
            RCLCPP_INFO(this->get_logger(), "Trying to open with device ID %d ", temp_device_id);
            _cap.open(temp_device_id, cv::CAP_V4L2);
            if (!_cap.isOpened()){
                RCLCPP_WARN(this->get_logger(), "Camera open with device ID %d failed", temp_device_id);
                RCLCPP_ERROR(this->get_logger(), "Please set the correct camera device ID");
                exit(1);
            }
        }
    }
    // 打印正确的 device_id；
    RCLCPP_INFO(this->get_logger(), "Camera opened success with device ID %d ! ! !", temp_device_id);
    // 如果打开相机的 device_id 与 config文件 中不一致，将正确的写入
    if (temp_device_id != _config.device_id) {
        YAML::Node temp_writer_yaml;
        temp_writer_yaml["device_id"] = temp_device_id;
        std::ofstream temp_writer_fstream("../config/normal_camera.yaml");
        temp_writer_fstream << temp_writer_yaml;
    }
    // 设置像素宽度;
    _cap.set(3, _config.frame_width);
    _cap.set(4, _config.frame_height);
}

void NormalCamera::PubImgCallback() {
    cv::Mat img;
    _cap.read(img);
    if (_config.undistort) {
        cv::undistort(img, img, _config.intrinsic, _config.dist_coeff);
    }
    sensor_msgs::msg::Image img_msg;
    _cv2msg.header.stamp = this->now();
    _cv2msg.image = img;
    _cv2msg.toImageMsg(img_msg);
    _iamge_pub->publish(img_msg);
    return;
}

