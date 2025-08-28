/*** 
 * @Author: 周健伟
 * @Date: 2025-08-28
 * @LastEditors: 周健伟
 * @LastEditTime: 2025-08-28
 * @Description: 
 * @
 * @Copyright (c) 2025 by 越甲灵动(杭州)科技有限公司, All Rights Reserved. 
 */

#include "image_task_core/base_detector.hpp"
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
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Detector : public rclcpp_lifecycle::LifecycleNode {    
public:
    Detector(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp_lifecycle::LifecycleNode("detector_node", options) {
        RCLCPP_INFO(this->get_logger(), "Hello detector_node !");

        flag_activate_ = false;
        this->declare_parameter<std::vector<std::string>>("detector_ids", {"dashboard_detector"});

        detector_loader_ = std::make_shared<pluginlib::ClassLoader<image_task_core::BaseDetector>>("image_task_core", "image_task_core::BaseDetector");        
        return;
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State & /*state*/) {
        RCLCPP_INFO(this->get_logger(), "Configuring...");
        flag_activate_ = false;
        std::vector<std::string> detector_ids = this->get_parameter("detector_ids").as_string_array();

        for (size_t i = 0; i < detector_ids.size(); i++) {
            std::shared_ptr<image_task_core::BaseDetector> detector = detector_loader_->createSharedInstance(detector_ids[i]);
            detector->configure(shared_from_this(), detector_ids[i]);
            detectors_.insert({detector_ids[i], detector});
        }

        object_detect_sub_ = this->create_service<interface::srv::ObjectDetect>("detect_object", std::bind(&Detector::object_detect_callback, this, _1, _2));
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & /*state*/) {
        flag_activate_ = true;
        return CallbackReturn::SUCCESS;
    }

    void object_detect_callback(const interface::srv::ObjectDetect::Request::SharedPtr request, const interface::srv::ObjectDetect::Response::SharedPtr response) {
        if (!flag_activate_) {
            RCLCPP_WARN(this->get_logger(), "detector_node haven't been activated!");
            return;
        }
        // todo: here...
        // detector_id
    }

    std::shared_ptr<pluginlib::ClassLoader<image_task_core::BaseDetector>> detector_loader_;
    std::map<std::string, std::shared_ptr<image_task_core::BaseDetector>> detectors_;
    rclcpp::Service<interface::srv::ObjectDetect>::SharedPtr object_detect_sub_;
    bool flag_activate_;
};

}

int main(int argc, char* argv[]) {

  // 1. 初始化 ROS 2
  rclcpp::init(argc, argv);
  
  // 2. 创建节点（此时节点处于 "unconfigured" 状态）
  auto node = std::make_shared<detector::Detector>();
  
  // 3. (可选但常见) 手动触发状态转换
  // 将节点从 unconfigured -> configuring -> inactive
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  
  // 将节点从 inactive -> activating -> active
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  
  // 4. 旋转节点，开始处理回调
  rclcpp::spin(node->get_node_base_interface());
  
  // 5. 关闭
  rclcpp::shutdown();
  
  return 0;
}


