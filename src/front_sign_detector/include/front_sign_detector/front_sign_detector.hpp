#ifndef FRONT_SIGN_DETECTOR_HPP
#define FRONT_SIGN_DETECTOR_HPP
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
#include <algorithm>


class front_sign_detector : public rclcpp::Node {
public:
    front_sign_detector(const rclcpp::NodeOptions& options);
private:
     struct candidateContour
	{
		std::vector<cv::Point> contour;
		double area = 0;
		cv::Point2f corner;
		candidateContour() = default;
		candidateContour(std::vector<cv::Point> contour) {
			this->contour = contour;
		}
	};

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr& msg);
    void processImage(const cv::Mat& image);
    void selectContours();
    void getCorners();
    void solveAngle();
    void show();
    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
    bool compareByRelativeAngle(const candidateContour& a, const candidateContour& b, const cv::Point2f& center);
	cv::Point2f computeCentroid(const std::vector<candidateContour>& candidates);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;
    rclcpp::Publisher<msg_interfaces::msg::SlotState>::SharedPtr  sucker_goal_publisher;
    rclcpp::Publisher<msg_interfaces::msg::SlotState>::SharedPtr slot_state_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
    geometry_msgs::msg::Pose last_frame_pose;
    int stable_frame_count = 0;
    cv::Mat CameraMatrix;
    cv::Mat DistCoeffs;
    cv::Mat source_image;
    cv::Mat processed_image;
    bool detect_color;
    int redThreshold = 120;
    int blueThreshold = 80;
    double minArea = 1000.0;
    double maxArea = 10000.0;
    double minSmallSquareArea = 300;
    double maxSmallSquareArea = 1000;
    std::vector<cv::Point> selected_contours;
    bool found = false;
    cv::Mat rVec;
    cv::Mat tVec;
    double camera_to_reference_x_offset = 0.0;
    double camera_to_reference_y_offset = -368.0;
    double camera_to_reference_z_offset = 0.0;


    //Four cornerPoints
	std::vector<candidateContour> corners;
	//Points from last frame
	std::vector<cv::Point> lastFrameCorners;
	//Points of current frame
	std::vector<cv::Point> currentFrameCorners;
	//Center of the four corner points
	cv::Point2f center;
	// Two small points
	std::vector<cv::Point2f> currentFrameSmallSquares;
	//Two small points in last frame
	std::vector<cv::Point2f> lastFrameSmallSquares;
	//Return the suitable set of contours

    class UnionFind {
	public:
		UnionFind(int size) {
			parent.resize(size);
			rank.resize(size, 0);
			for (int i = 0; i < size; ++i) {
				parent[i] = i;
			}
		}

		int find(int x) {
			if (parent[x] != x) {
				parent[x] = find(parent[x]);
			}
			return parent[x];
		}

		void unite(int x, int y) {
			int rootX = find(x);
			int rootY = find(y);
			if (rootX != rootY) {
				if (rank[rootX] < rank[rootY]) {
					parent[rootX] = rootY;
				}
				else if (rank[rootX] > rank[rootY]) {
					parent[rootY] = rootX;
				}
				else {
					parent[rootY] = rootX;
					rank[rootX]++;
				}
			}
		}

		bool same(int x, int y) {
			return find(x) == find(y);
		}
	private:
		std::vector<int> parent;
		std::vector<int> rank;
	};
};
#endif