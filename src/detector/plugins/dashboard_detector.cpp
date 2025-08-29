#include "detector/plugins/dashboard_detector.hpp"

#include <chrono>
#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"


namespace detector {

void DashboardDetector::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr parent, const std::string & detector_id) {
    node_ = parent;

    node_->declare_parameter<std::string>(detector_id + ".det_param_package", "detector");
    node_->declare_parameter<std::string>(detector_id + ".det_bin_package", "detector");
    node_->declare_parameter<std::string>(detector_id + ".yolov8_param_package", "detector");
    node_->declare_parameter<std::string>(detector_id + ".yolov8_bin_package", "detector");
    node_->declare_parameter<std::string>(detector_id + ".det_param", "nanodet.param");
    node_->declare_parameter<std::string>(detector_id + ".det_bin", "nanodet.bin");
    node_->declare_parameter<std::string>(detector_id + ".yolov8_param", "yolov8s-pose-opt.param");
    node_->declare_parameter<std::string>(detector_id + ".yolov8_bin", "yolov8s-pose-opt.bin");
    
    std::string det_param_package = node_->get_parameter(detector_id + ".det_param_package").as_string();
    std::string det_bin_package = node_->get_parameter(detector_id + ".det_bin_package").as_string();
    std::string yolov8_param_package = node_->get_parameter(detector_id + ".yolov8_param_package").as_string();
    std::string yolov8_bin_package = node_->get_parameter(detector_id + ".yolov8_bin_package").as_string();
    std::string det_param = node_->get_parameter(detector_id + ".det_param").as_string();
    std::string det_bin = node_->get_parameter(detector_id + ".det_bin").as_string();
    std::string yolov8_param = node_->get_parameter(detector_id + ".yolov8_param").as_string();
    std::string yolov8_bin = node_->get_parameter(detector_id + ".yolov8_bin").as_string();
    
    std::string det_param_path = ament_index_cpp::get_package_share_directory(det_param_package) + "/" + det_param;
    std::string det_bin_path = ament_index_cpp::get_package_share_directory(det_bin_package) + "/" + det_bin;
    std::string yolov8_param_path = ament_index_cpp::get_package_share_directory(yolov8_param_package) + "/" + yolov8_param;
    std::string yolov8_bin_path = ament_index_cpp::get_package_share_directory(yolov8_bin_package) + "/" + yolov8_bin;
    
    nanoDet_ = std::make_unique<NanoDet>(det_param_path.c_str(), det_bin_path.c_str(), false);
    yolov8Pose_ = std::make_unique<Yolov8Pose>(yolov8_param_path.c_str(), yolov8_bin_path.c_str(), false);

}

std::optional<std::vector<interface::msg::Object>> DashboardDetector::detect(const cv::Mat& input_image) {
    if (input_image.empty()) {
        std::cerr << "cv::imread read image failed" << std::endl;
        return std::nullopt;
    }

    std::vector<ObjectDetect> objects;
    object_rect effect_roi;
    cv::Mat resized_img;
    nanoDet_->resize_uniform(input_image, resized_img, cv::Size(320, 320), effect_roi);
    auto results = nanoDet_->detect(resized_img, 0.3, 0.3);  // 0.4, 0.3
    nanoDet_->convert_boxes(results, effect_roi, input_image.cols, input_image.rows, objects);

    std::cout << "Object size: " << objects.size() << std::endl;

    std::cout << "object[0].prob: " << objects[0].prob << std::endl;

    cv::rectangle(input_image, cv::Rect(objects[0].rect), cv::Scalar(0, 255, 0), 2);

    cv::imshow("Drawing Float Rectangle", input_image);
    cv::waitKey(0);

    if (!objects.empty() && yolov8Pose_->isValidROI(objects)) {
        std::vector<cv::Mat> meters_image = yolov8Pose_->cut_roi_img(input_image, objects);
        std::vector<float> scale_values;
        bool isGet = yolov8Pose_->get_results(meters_image, scale_values);

        // debug:
        std::cout << "debug.scale_values: " << scale_values[0] << std::endl;
        // debug
        
        if (isGet)
        {
            for (const auto& scale_value:scale_values)
            {
                if (scale_value <= 0.50f)
                {
                    // result = scale_values[i_results] + 0.012f;  // 0.01f
                    std::cout << "scale_value: " << yolov8Pose_->floatToString(scale_value + 0.012f) + " Mpa" << std::endl;

                    // debug:
                    std::cout << "scale_value <= 0.50f" << std::endl;
                    // debug
                }
                else
                {
                    // result = scale_values[i_results] + 0.008f;  // 0.08f
                    std::cout << "scale_value: " << yolov8Pose_->floatToString(scale_value + 0.008f) + " Mpa" << std::endl;
                    
                    // debug:
                    std::cout << "scale_value >= 0.50f" << std::endl;
                    // debug
                }
            }
        } else {
            std::cout << "No objects detected." << std::endl;
        }

    } else {
        std::cout << "No objects detected." << std::endl;
    }

    // todo:
    std::vector<interface::msg::Object> test_object;
    return test_object;
}

DashboardDetector::~DashboardDetector() {

}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(detector::DashboardDetector, image_task_core::BaseDetector)
