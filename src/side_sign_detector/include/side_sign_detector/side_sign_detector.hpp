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
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
class side_sign_detector : public rclcpp::Node {
public:
    side_sign_detector(const rclcpp::NodeOptions& options);
    void synced_callback(const sensor_msgs::msg::Image::SharedPtr& image,
                         const sensor_msgs::msg::Image::SharedPtr& depth);
    void processImage(const cv::Mat& image, const cv::Mat& depth);
    void select_contours();
    void solve_angle();

private:
    float get_depth(int x, int y);
    cv::Point3f deproject_pixel_to_point(int u, int v, float depth);
    message_filters::Subscriber<sensor_msgs::msg::Image> image_subscription;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_subscription;
    rclcpp::Publisher<msg_interfaces::msg::SlotState>::SharedPtr slot_state_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;
    geometry_msgs::msg::Pose last_frame_pose;
    int stable_frame_count = 0;
    cv::Mat CameraMatrix;
    cv::Mat DistCoeffs;
    cv::Mat D435i_CameraMatrix;
    cv::Mat depth_image;
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
