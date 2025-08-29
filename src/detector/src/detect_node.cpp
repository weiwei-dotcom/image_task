/*** 
 * @Author: 周健伟
 * @Date: 2025-08-28
 * @LastEditors: 周健伟
 * @LastEditTime: 2025-08-29
 * @Description: 
 * @
 * @Copyright (c) 2025 by 越甲灵动(杭州)科技有限公司, All Rights Reserved. 
 */

#include "image_task_core/base_detector.hpp"
#include "detector/detect_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "interface/srv/object_detect.hpp"
#include "interface/msg/object.hpp"
#include "detector/nanodet.h"
#include <chrono>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace detector {

using namespace std::placeholders;

Detector::Detector(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode("detect_node", options) {
    RCLCPP_INFO(this->get_logger(), "Hello detector_node !");

    flag_activate_ = false;
    this->declare_parameter<std::vector<std::string>>("detector_ids", {"dashboard_detector"});

    detector_loader_ = std::make_shared<pluginlib::ClassLoader<image_task_core::BaseDetector>>("image_task_core", "image_task_core::BaseDetector");        
    return;
}

CallbackReturn Detector::on_configure(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(this->get_logger(), "Configuring...");
    flag_activate_ = false;
    std::vector<std::string> detector_ids = this->get_parameter("detector_ids").as_string_array();
    for (size_t i = 0; i < detector_ids.size(); i++) {
        std::string detector_plugin = this->get_parameter(detector_ids[i]+".plugin").as_string();
        std::shared_ptr<image_task_core::BaseDetector> detector = detector_loader_->createSharedInstance(detector_plugin);
        detector->configure(shared_from_this(), detector_ids[i]);
        detectors_.insert({detector_ids[i], detector});
    }
    object_detect_sub_ = this->create_service<interface::srv::ObjectDetect>("detect_object", std::bind(&Detector::object_detect_callback, this, _1, _2));
    return CallbackReturn::SUCCESS;
}
CallbackReturn Detector::on_activate(const rclcpp_lifecycle::State & /*state*/) {
    flag_activate_ = true;
    return CallbackReturn::SUCCESS;
}
CallbackReturn Detector::on_deactivate(const rclcpp_lifecycle::State & /*state*/) {
    flag_activate_ = false;
    return CallbackReturn::SUCCESS;
}

CallbackReturn Detector::on_shutdown(const rclcpp_lifecycle::State & /*state*/) {
    flag_activate_ = false;
    detector_loader_.reset();
    return CallbackReturn::SUCCESS;
}
CallbackReturn Detector::on_cleanup(const rclcpp_lifecycle::State & /*state*/) {
    flag_activate_ = false;
    object_detect_sub_.reset();
    detectors_.clear();
    return CallbackReturn::SUCCESS;
}

void Detector::object_detect_callback(const interface::srv::ObjectDetect::Request::SharedPtr request, const interface::srv::ObjectDetect::Response::SharedPtr response) {
    if (!flag_activate_) {
        RCLCPP_WARN(this->get_logger(), "detector_node haven't been activated!");
        return;
    }
    std::string detect_id = request->detector_id;
    if (detect_id.empty()) {
        detect_id = "dashboard_detector";
    }
    if (detectors_.find(detect_id) == detectors_.end()) {
        RCLCPP_ERROR(this->get_logger(), "%s you specified is not valid", detect_id.c_str());
        return;
    }
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(request->image, request->image.encoding);
    cv::Mat input_img = cv_ptr->image;
    auto detect_result = detectors_[detect_id]->detect(input_img);
    if (!detect_result.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "object detect failed !");
        response->success = false;
        return;
    }

    std::vector<interface::msg::Object> objects = detect_result.value();
    // todo: 
    response->success = true;
    response->objects = objects;
}

Detector::~Detector() {
    //todo:
}

};

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<detector::Detector>();
  
  // 3. (可选但常见) 手动触发状态转换
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  
  rclcpp::spin(node->get_node_base_interface());
  
  rclcpp::shutdown();
  
  return 0;
}


