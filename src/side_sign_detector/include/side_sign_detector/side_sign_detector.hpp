//
// Created by stardust on 2025/1/2.
//

#ifndef BUILD_SIDE_SIGN_DETECTOR_HPP
#define BUILD_SIDE_SIGN_DETECTOR_HPP
#define RED 0
#define BLUE 1

#include <opencv4/opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <cv_bridge/cv_bridge.h>
#include "rclcpp_components/register_node_macro.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>



class side_sign_detector : public rclcpp::Node {
public:
    side_sign_detector(const rclcpp::NodeOptions& options);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr& msg);
    void processImage(const cv::Mat& image);
    void select_contours();
    void solve_angle();
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;
    cv::Mat CameraMatrix;
    cv::Mat DistCoeffs;
    cv::Mat source_image;
    cv::Mat processed_image;
    bool detect_color;
    int redThreshold = 160;
    int blueThreshold = 80;
    int minArea = 1000;
    std::vector<cv::Point> selected_contours;
};


#endif //BUILD_SIDE_SIGN_DETECTOR_HPP