//
// Created by engineer on 2025/3/9.
//
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#ifndef BUILD_MINE_DETECTOR_HPP
#define BUILD_MINE_DETECTOR_HPP
class mine_detector : public rclcpp::Node {
public:
    mine_detector(const rclcpp::NodeOptions& options);
private:
    cv::dnn::Net model;
    std::vector<std::string> classes;
    float confThreshold = 0.5;
    float nmsThreshold = 0.4;

    void loadModel();
    std::vector<cv::Rect> detectROI(const cv::Mat& image);
    void processROI(cv::Mat& image, const cv::Rect& roi);
    void showResults(const cv::Mat& image);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;
    cv::Mat preProcessImage(const cv::Mat& image, const cv::Size& size = cv::Size(640, 480));
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr& msg);
};

#endif //BUILD_MINE_DETECTOR_HPP
