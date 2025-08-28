/*** 
 * @Author: 周健伟
 * @Date: 2025-08-28
 * @LastEditors: 周健伟
 * @LastEditTime: 2025-08-28
 * @Description: 
 * @
 * @Copyright (c) 2025 by 越甲灵动(杭州)科技有限公司, All Rights Reserved. 
 */
/*** 
 * @Author: 周健伟
 * @Date: 2025-08-28
 * @LastEditors: 周健伟
 * @LastEditTime: 2025-08-28
 * @Description: 
 * @
 * @Copyright (c) 2025 by 越甲灵动(杭州)科技有限公司, All Rights Reserved. 
 */
#include "detector/yolov8_pose.h"
#include "detector/nanodet.h"

#include <chrono>
#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "interface/msg/object_dashboard.hpp"
#include "image_task_core/base_detector.hpp"


namespace detector {

class DashboardDetector : public image_task_core::BaseDetector {

public:
    PureSpinController() = default;

    void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr parent, const std::string & detector_id) override;

    interface::msg::ObjectDashboard detect(const cv::Mat& input_image) override;

    rclcpp_lifecycle::Node::SharedPtr node_;
    std::unique_ptr<NanoDet> nanoDet;
    std::unique_ptr<Yolov8Pose> yolov8Pose;
}

}

