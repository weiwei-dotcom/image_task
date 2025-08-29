/*** 
 * @Author: 周健伟
 * @Date: 2025-08-28
 * @LastEditors: 周健伟
 * @LastEditTime: 2025-08-29
 * @Description: 
 * @
 * @Copyright (c) 2025 by 越甲灵动(杭州)科技有限公司, All Rights Reserved. 
 */

#pragma once
#ifndef DETECTOR__DETECTOR_NODE__HPP
#define DETECTOR__DETECTOR_NODE__HPP

#include "image_task_core/base_detector.hpp"
#include "pluginlib/class_loader.hpp"
#include "interface/msg/object.hpp"
#include "cv_bridge/cv_bridge.h"
#include "interface/srv/object_detect.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <chrono>
#include <iostream>
#include <vector>

namespace detector {

using namespace std::placeholders;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Detector : public rclcpp_lifecycle::LifecycleNode {   
public:
    Detector(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~Detector();

protected: 
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

private:
    void object_detect_callback(const interface::srv::ObjectDetect::Request::SharedPtr request, const interface::srv::ObjectDetect::Response::SharedPtr response);

    rclcpp::Service<interface::srv::ObjectDetect>::SharedPtr object_detect_sub_;
    std::shared_ptr<pluginlib::ClassLoader<image_task_core::BaseDetector>> detector_loader_;
    std::map<std::string, std::shared_ptr<image_task_core::BaseDetector>> detectors_;
    bool flag_activate_;
};

}


#endif