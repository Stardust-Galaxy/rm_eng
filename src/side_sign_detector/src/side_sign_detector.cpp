//
// Created by stardust on 2025/1/2.
//

#include "../include/side_sign_detector/side_sign_detector.hpp"

side_sign_detector::side_sign_detector(const rclcpp::NodeOptions &options) : Node("side_sign_detector", options) {
    CameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    DistCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    try {
        std::string current_path = std::filesystem::current_path();
        std::string yaml_file_path = current_path + "/src/side_sign_detector/config/camera.yaml";
        YAML::Node config = YAML::LoadFile(yaml_file_path);

        const YAML::Node& camera_matrix_node = config["camera_matrix"]["data"];
        int index = 0;
        for(const auto& value : camera_matrix_node) {
            CameraMatrix.at<double>(index) = value.as<double>();
            index++;
        }

        const YAML::Node& dist_coeffs_node = config["distortion_coefficients"];
        index = 0;
        for(const auto& value : dist_coeffs_node) {
            DistCoeffs.at<double>(index) = value.as<double>();
            index++;
        }

        detect_color = config["detect_blue_color"].as<bool>();
    } catch (const YAML::ParserException& e) {
        std::cerr << "YAML parsing Error: " << e.what() << std::endl;
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Filesystem Error: " << e.what() << std::endl;
    } catch (const std::exception & e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    slot_state_publisher = this->create_publisher<msg_interfaces::msg::SlotState>("slot_state", rclcpp::QoS(10));
    {
        last_frame_pose.position.x = 0.0;
        last_frame_pose.position.y = 0.0;
        last_frame_pose.position.z = 0.0;
        last_frame_pose.orientation.x = 0.0;
        last_frame_pose.orientation.y = 0.0;
        last_frame_pose.orientation.z = 0.0;
        last_frame_pose.orientation.w = 1.0;
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
    //cv::imshow("image", img);
    //cv::waitKey(1);
    processImage(img);
    select_contours();
    solve_angle();
}

void side_sign_detector::processImage(const cv::Mat &image) {
        source_image = image;
        cv::Mat gray_image;
        cv::Mat binary_result;
        std::vector<cv::Mat> channels;
        cv::split(source_image,channels);
        cv::Mat red_channel = channels[2];
        cv::Mat blue_channel = channels[0];
        std::vector<cv::Mat> merged_channels;
        cv::Mat merged_image;
        merged_channels.push_back(blue_channel);
        merged_channels.push_back(cv::Mat::zeros(blue_channel.size(), CV_8UC1));
        merged_channels.push_back(red_channel);
        cv::merge(merged_channels, merged_image);

        cv::cvtColor(merged_image, gray_image, cv::COLOR_BGR2GRAY);
        if(detect_color == RED)
            cv::threshold(gray_image, binary_result, redThreshold, 255, cv::THRESH_BINARY);
        else if(detect_color == BLUE)
            cv::threshold(gray_image, binary_result, blueThreshold, 255, cv::THRESH_BINARY);

        //cv::imshow("binary result", binary_result);
        //cv::waitKey(1);

        cv::Mat denoised_result;
        int kernel_size = 5 ;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
        cv::erode(binary_result, denoised_result, kernel);
        cv::dilate(denoised_result, denoised_result, kernel, cv::Point(-1,-1) ,2);

        cv::imshow("denoised result", denoised_result);
        cv::waitKey(1);

        cv::Mat edges_result;
        cv::Canny(denoised_result, edges_result, 50, 100);
        processed_image = edges_result;

}

void side_sign_detector::select_contours() {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(processed_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    found = false;
    for(const auto & contour : contours) {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, cv::arcLength(contour, true) * 0.02, true);
        if((approx.size() <= 6 && approx.size() >= 9) || cv::contourArea(contour) < minArea || cv::contourArea(contour) > maxArea) {
            continue;
        }
        selected_contours = contour;
        //cv::drawContours(source_image, contours, -1, cv::Scalar(0, 255, 0), 2);
        //print the area next to the contour
        //find the origin of the contour
        cv::Point origin = contour[0];
        for(const auto & point : contour) {
            if(point.x < origin.x) {
                origin = point;
            }
        }
        cv::putText(source_image, "area:" + std::to_string(cv::contourArea(contour)), origin, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        found = true;
    }
    //cv::imshow("result", source_image);
    //cv::waitKey(1);
}

void side_sign_detector::solve_angle() {
    if(!found) {
        cv::putText(source_image, "pitch: unknown", cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "yaw: unknown", cv::Point(20, 110), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "roll: unknown", cv::Point(20, 160), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "reference_pitch: unknown", cv::Point(20, 210), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "reference_yaw: unknown", cv::Point(20, 260), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "reference_roll: unknown", cv::Point(20, 310), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "reference_x: unknown", cv::Point(20, 510), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "reference_y: unknown", cv::Point(20, 550), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "reference_z: unknown", cv::Point(20, 610), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "x: unknown", cv::Point(20, 360), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "y: unknown", cv::Point(20, 410), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "z: unknown", cv::Point(20, 460), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::imshow("result", source_image);
        cv::waitKey(1);
        sqpnp_initialized = false;
        stable_frame_count = 0;
        return;
    }
    //camera frame to reference frame
    cv::Mat camera_to_reference_rVec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat camera_to_reference_tVec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat camera_to_reference_rMat = cv::Mat::eye(3, 3, CV_64F);
    //Three transformation matrix
    cv::Mat T_camera_to_reference = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat T_world_to_reference = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat T_world_to_camera = cv::Mat::eye(4, 4, CV_64F);
    camera_to_reference_tVec.at<double>(0,0) = camera_to_reference_x_offset;
    camera_to_reference_tVec.at<double>(1,0) = camera_to_reference_y_offset;
    camera_to_reference_tVec.at<double>(2,0) = camera_to_reference_z_offset;
    cv::Rodrigues(camera_to_reference_rVec, camera_to_reference_rMat);
    camera_to_reference_rMat.copyTo(T_camera_to_reference(cv::Rect(0, 0, 3, 3)));
    camera_to_reference_tVec.copyTo(T_camera_to_reference(cv::Rect(3, 0, 1, 3)));
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
    //draw out the triangle
    cv::line(source_image, approx_triangle[0], approx_triangle[1], cv::Scalar(0, 255, 0), 2);
    cv::line(source_image, approx_triangle[1], approx_triangle[2], cv::Scalar(0, 255, 0), 2);
    cv::line(source_image, approx_triangle[2], approx_triangle[0], cv::Scalar(0, 255, 0), 2);

    //point out the max point
    cv::circle(source_image, max_angle_point, 5, cv::Scalar(0, 0, 255), 2);
    cv::Point upper_point;
    cv::Point lower_point;
    for(size_t i = 0; i < approx_triangle.size(); i++) {
        if(approx_triangle[i].y < max_angle_point.y) {
            upper_point = approx_triangle[i];
        } else {
            lower_point = approx_triangle[i];
        }
    }
    cv::circle(source_image, upper_point, 5, cv::Scalar(0, 255, 0), 2);
    cv::circle(source_image, lower_point, 5, cv::Scalar(255, 0, 0), 2);
    if(max_angle_point.x < upper_point.x) {
        direction = LEFT_ORIENTATION;
    } else {
        direction = RIGHT_ORIENTATION;
    }
    std::vector<cv::Point3f> object_points;
    //use slot center for solvePnP
    if(direction == LEFT_ORIENTATION) {
        object_points = {
                cv::Point3f(144, -100, -1.5),
                cv::Point3f(144, 0, 98.5),
                cv::Point3f(144, 100, -1.5)
        };
    } else {
        object_points = {
                cv::Point3f(-144, -100, -1.5),
                cv::Point3f(-144, 0, 98.5),
                cv::Point3f(-144, 100, -1.5)
        };
    }
    std::vector<cv::Point2f> image_points = {
            cv::Point2f(upper_point.x, upper_point.y),
            cv::Point2f(max_angle_point.x, max_angle_point.y),
            cv::Point2f(lower_point.x, lower_point.y)
    };
    if(!sqpnp_initialized) {
        cv::Mat tVec, rVec;
        cv::solvePnP(object_points, image_points, CameraMatrix, DistCoeffs, rVec, tVec, false, cv::SOLVEPNP_SQPNP);
        this->tVec = tVec;
        this->rVec = rVec;
        sqpnp_initialized = true;
    }
    else {
        cv::solvePnP(object_points, image_points, CameraMatrix, DistCoeffs, this->rVec, this->tVec, true, cv::SOLVEPNP_ITERATIVE);
        cv::Mat rotation_matrix;
        cv::Rodrigues(this->rVec, rotation_matrix);
        rotation_matrix.copyTo(T_world_to_camera(cv::Rect(0, 0, 3, 3)));
        this->tVec.copyTo(T_world_to_camera(cv::Rect(3, 0, 1, 3)));
        T_world_to_reference = T_camera_to_reference.inv() * T_world_to_camera;
        cv::Mat world_to_reference_rMat = T_world_to_reference(cv::Rect(0, 0, 3, 3));
        cv::Mat world_to_reference_tVec = T_world_to_reference(cv::Rect(3, 0, 1, 3));
        cv::Mat mtxR, mtxQ;
        cv::Vec3d euler_angles = cv::RQDecomp3x3(rotation_matrix, mtxR, mtxQ, cv::noArray(),cv::noArray());
        if (euler_angles[0] > 0 || abs(euler_angles[0]) > 90 || abs(euler_angles[1]) > 135 ) {
            sqpnp_initialized = false;
            return;
        }
        cv::Vec3d reference_euler_angles = cv::RQDecomp3x3(world_to_reference_rMat, mtxR, mtxQ, cv::noArray(),cv::noArray());
        double reference_x = world_to_reference_tVec.at<double>(0, 0);
        double reference_y = world_to_reference_tVec.at<double>(1, 0);
        double reference_z = world_to_reference_tVec.at<double>(2, 0);
        double reference_pitch = reference_euler_angles[0];
        double reference_yaw = reference_euler_angles[1];
        double reference_roll = reference_euler_angles[2];
        //print euler angle on the screen through imshow
        cv::putText(source_image, "pitch:" + std::to_string(euler_angles[0]), cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "yaw:" + std::to_string(euler_angles[1]), cv::Point(20, 110), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "roll:" + std::to_string(euler_angles[2]), cv::Point(20, 160), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "reference_pitch:" + std::to_string(reference_euler_angles[0]), cv::Point(20, 210), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "reference_yaw:" + std::to_string(reference_euler_angles[1]), cv::Point(20, 260), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "reference_roll:" + std::to_string(reference_euler_angles[2]), cv::Point(20, 310), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "reference_x:" + std::to_string(world_to_reference_tVec.at<double>(0, 0)), cv::Point(20, 510), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "reference_y:" + std::to_string(world_to_reference_tVec.at<double>(1, 0)), cv::Point(20, 550), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "reference_z:" + std::to_string(world_to_reference_tVec.at<double>(2, 0)), cv::Point(20, 610), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "x:" + std::to_string(this->tVec.at<double>(0, 0)), cv::Point(20, 360), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "y:" + std::to_string(this->tVec.at<double>(1, 0)), cv::Point(20, 410), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "z:" + std::to_string(this->tVec.at<double>(2, 0)), cv::Point(20, 460), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "direction:" + ((direction == LEFT_ORIENTATION) ? std::string("LEFT_ORIENTED") : std::string("RIGHT_ORIENTED")), cv::Point(20, 660), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(source_image, "stabilization_state:" + ((stable_frame_count > 20) ? std::string("STABLE") : std::string("UNSTABLE")), cv::Point(20, 710), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::imshow("result", source_image);
        cv::waitKey(1);
        //wait until the slot stops moving
        if(abs(reference_x - last_frame_pose.position.x ) < 0.5
           && abs(reference_y - last_frame_pose.position.y) < 0.5
           && abs(reference_z - last_frame_pose.position.z) < 0.5
                ) {
            stable_frame_count++;
        } else {
            stable_frame_count = 0;
        }
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "base_link";
        pose_msg.pose.position.x = world_to_reference_tVec.at<double>(0, 0);
        pose_msg.pose.position.y = world_to_reference_tVec.at<double>(1, 0);
        pose_msg.pose.position.z = world_to_reference_tVec.at<double>(2, 0);
        tf2::Quaternion q;
        q.setRPY(-reference_yaw, reference_roll, reference_pitch);
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
        msg_interfaces::msg::SlotState current_slot_state;
        current_slot_state.pose = pose_msg;
        current_slot_state.slot_stabled = static_cast<int8_t >(stable_frame_count > 20);
        last_frame_pose = pose_msg.pose;
        slot_state_publisher->publish(current_slot_state);
    }


}
RCLCPP_COMPONENTS_REGISTER_NODE(
        side_sign_detector
        )