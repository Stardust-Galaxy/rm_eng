//
// Created by stardust on 2025/1/2.
//

#include "../include/side_sign_detector/side_sign_detector.hpp"

side_sign_detector::side_sign_detector(const rclcpp::NodeOptions &options) : Node("side_sign_detector", options) {
    CameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    DistCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    try {
        std::string current_path = std::filesystem::current_path();
        std::string yaml_file_path = current_path + "/src/side_sign_detector/config/camera_info.yaml";
        YAML::Node config = YAML::LoadFile(yaml_file_path);

        const YAML::Node& camera_matrix_node = config["camera_matrix"]["data"];
        int index = 0;
        for(const auto& value : camera_matrix_node) {
            CameraMatrix.at<double>(index) = value.as<double>();
            index++;
        }

        const YAML::Node& dist_coeffs_node = config["distortion_coefficients"]["data"];
        index = 0;
        for(const auto& value : dist_coeffs_node) {
            DistCoeffs.at<double>(index) = value.as<double>();
            index++;
        }

        detect_color = config["detect_red_color"].as<bool>();
    } catch (const YAML::ParserException& e) {
        std::cerr << "YAML parsing Error: " << e.what() << std::endl;
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Filesystem Error: " << e.what() << std::endl;
    } catch (const std::exception & e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    image_subscription = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                imageCallback(msg);
            });
}

void side_sign_detector::imageCallback(const sensor_msgs::msg::Image::SharedPtr &msg) {
    auto img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    processImage(img);
    select_contours();

}

void side_sign_detector::processImage(const cv::Mat &image) {
        source_image = image;
        cv::Mat gray_image;
        cv::Mat binary_result;
        std::vector<cv::Mat> channels;
        cv::Mat red_channel = channels[2];
        cv::Mat blue_channel = channels[0];
        std::vector<cv::Mat> merged_channels;
        cv::Mat merged_image;
        merged_channels.push_back(red_channel);
        merged_channels.push_back(cv::Mat::zeros(blue_channel.size(), CV_8UC1));
        merged_channels.push_back(blue_channel);
        cv::merge(merged_channels, merged_image);
        cv::cvtColor(merged_image, gray_image, cv::COLOR_BGR2GRAY);
        if(detect_color == RED)
            cv::threshold(gray_image, binary_result, redThreshold, 255, cv::THRESH_BINARY);
        else if(detect_color == BLUE)
            cv::threshold(gray_image, binary_result, blueThreshold, 255, cv::THRESH_BINARY);
        cv::Mat denoised_result;
        int kernel_size =3 ;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
        cv::erode(binary_result, denoised_result, kernel);
        cv::dilate(denoised_result, denoised_result, kernel);
        cv::Mat edges_result;
        cv::Canny(denoised_result, edges_result, 50, 100);
        cv::imshow("result", edges_result);
        cv::waitKey(1);
        processed_image = edges_result;
}

void side_sign_detector::select_contours() {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(processed_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for(size_t i = 0;i < contours.size(); i++) {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[i], approx, cv::arcLength(contours[i], true) * 0.02, true);
        if(approx.size() <= 5 && approx.size() >= 9 ) {
            continue;
        }
        selected_contours = contours[i];
        break;
    }
}

void side_sign_detector::solve_angle() {
    double max_angle = 0;
    cv::Point max_angle_point;
    std::vector<cv::Point> approx_triangle;
    cv::minEnclosingTriangle(selected_contours, approx_triangle);
    for(size_t i = 0; i < approx_triangle.size(); i++) {
        const cv::Point& a = approx_triangle[i];
        const cv::Point& b = approx_triangle[(i + 1) % 3];
        const cv::Point& c = approx_triangle[(i + 2) % 3];

        cv::Point ab = { a.x - b.x, a.y - b.y };
        cv::Point cb = { c.x - b.x, c.y - b.y };
        // tan = crossProduct / dotProduct
        double dotProduct = ab.x * cb.x + ab.y * cb.y;
        double crossProduct = ab.x * cb.y - ab.y * cb.x;
        double angle = std::atan(std::abs(crossProduct) / std::abs(dotProduct));
        if (angle > max_angle) {
            max_angle = angle;
            max_angle_point = b;
        }
    }
    cv::Point upper_point;
    cv::Point lower_point;
    for(size_t i = 0; i < approx_triangle.size() && approx_triangle[i] != max_angle_point; i++) {
        if(approx_triangle[i].y < max_angle_point.y) {
            upper_point = approx_triangle[i];
        } else {
            lower_point = approx_triangle[i];
        }
    }
    std::vector<cv::Point3f> object_points = {
            cv::Point3f(-0.05, -0.1, 0),
            cv::Point3f(0.05, 0, 0),
            cv::Point3f(-0.05, 0.1, 0)
    };
    std::vector<cv::Point2f> image_points = {
            cv::Point2f(upper_point.x, upper_point.y),
            cv::Point2f(max_angle_point.x, max_angle_point.y),
            cv::Point2f(lower_point.x, lower_point.y)
    };
    cv::Mat tVec, rVec;
    cv::solvePnP(object_points, image_points, CameraMatrix, DistCoeffs, rVec, tVec);
}
RCLCPP_COMPONENTS_REGISTER_NODE(
        side_sign_detector
        )