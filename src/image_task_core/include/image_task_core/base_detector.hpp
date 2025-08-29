/*** 
 * @Author: 周健伟
 * @Date: 2025-08-28
 * @LastEditors: 周健伟
 * @LastEditTime: 2025-08-28
 * @Description: 
 * @
 * @Copyright (c) 2025 by 越甲灵动(杭州)科技有限公司, All Rights Reserved. 
 */

#pragma once
#ifndef BASE_DETECTOR__HPP
#define BASE_DETECTOR__HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "interface/msg/object.hpp"
#include "opencv2/opencv.hpp"

namespace image_task_core {

class BaseDetector {
public:
    BaseDetector() = default;
    virtual void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr parent, const std::string & detector_id) = 0;
    virtual std::optional<std::vector<interface::msg::Object>> detect(const cv::Mat & input_image) = 0;
    ~BaseDetector() = default;

};
    
}


#endif