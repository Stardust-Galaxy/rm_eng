//
// Created by stardust on 2025/1/2.
//

#ifndef BUILD_SIDE_SIGN_DETECTOR_HPP
#define BUILD_SIDE_SIGN_DETECTOR_HPP
#define RED 0
#define BLUE 1
#define LEFT_ORIENTATION 0
#define RIGHT_ORIENTATION 1
#include <opencv4/opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <tf2/LinearMath/Quaternion.h>
#include <msg_interfaces/msg/slot_state.hpp>

class side_sign_detector : public rclcpp::Node {
public:
    side_sign_detector(const rclcpp::NodeOptions& options);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr& msg);
    void processImage(const cv::Mat& image);
    void select_contours();
    void solve_angle();
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;
    rclcpp::Publisher<msg_interfaces::msg::SlotState>::SharedPtr slot_state_publisher;
    geometry_msgs::msg::Pose last_frame_pose;
    int stable_frame_count = 0;
    cv::Mat CameraMatrix;
    cv::Mat DistCoeffs;
    cv::Mat source_image;
    cv::Mat processed_image;
    bool detect_color;
    int redThreshold = 30;
    int blueThreshold = 80;
    double minArea = 8000.0;
    double maxArea = 35000.0;
    std::vector<cv::Point> selected_contours;
    bool found = false;
    bool direction;
    double camera_to_reference_x_offset = 0.0;
    double camera_to_reference_y_offset = 0.0;
    double camera_to_reference_z_offset = 0.0;
    bool sqpnp_initialized = false;
    cv::Mat rVec;
    cv::Mat tVec;
};


#endif //BUILD_SIDE_SIGN_DETECTOR_HPP
