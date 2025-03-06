//
// Created by stardust on 2025/1/2.
//

#include "../include/side_sign_detector/side_sign_detector.hpp"

side_sign_detector::side_sign_detector(const rclcpp::NodeOptions &options) : Node("side_sign_detector", options) {
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    CameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    DistCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    D435i_CameraMatrix = cv::Mat::eye(3, 3, CV_64F);
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

        const YAML::Node& dist_coeffs_node = config["distortion_coeff"];
        index = 0;
        for(const auto& value : dist_coeffs_node) {
            DistCoeffs.at<double>(index) = value.as<double>();
            index++;
        }

        const YAML::Node& d435i_camera_matrix_node = config["d435i_camera_matrix"]["data"];
        index = 0;
        for(const auto& value : d435i_camera_matrix_node) {
            D435i_CameraMatrix.at<double>(index) = value.as<double>();
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
    image_subscription.subscribe(this, "/camera/color/image_raw", rmw_qos_profile_sensor_data);
    depth_subscription.subscribe(this, "/camera/camera/depth/image_rect_raw", rmw_qos_profile_sensor_data);
    auto sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), image_subscription, depth_subscription);
    sync->registerCallback(&side_sign_detector::synced_callback, this);
}

void side_sign_detector::synced_callback(const sensor_msgs::msg::Image::SharedPtr &image,
                                         const sensor_msgs::msg::Image::SharedPtr &depth) {
    auto color_image = cv_bridge::toCvCopy(image, "bgr8")->image;
    auto depth_image = cv_bridge::toCvCopy(depth, "32FC1")->image;
    //cv::imshow("image", color_image);
    //cv::waitKey(1);
    processImage(color_image, depth_image);
    select_contours();
    solve_angle();
}

void side_sign_detector::processImage(const cv::Mat &image, const cv::Mat &depth) {
        depth_image = depth;
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
    std::vector<cv::Point3f> camera_points;
    for(const auto& point : image_points) {
        int u = static_cast<int>(point.x);
        int v = static_cast<int>(point.y);
        float depth = get_depth(u, v);
        if(depth > 0) {
            cv::Point3f camera_point = deproject_pixel_to_point(u, v, depth);
            camera_points.push_back(camera_point);
        }
    }
    if (camera_points.size() != object_points.size()) {
        RCLCPP_WARN(this->get_logger(), "Mismatch in number of camera and object points");
        sqpnp_initialized = false;
        return;
    }

    // Convert points to Eigen matrices for SVD
    Eigen::MatrixXd cam_pts(3, camera_points.size());
    Eigen::MatrixXd obj_pts(3, object_points.size());
    for (size_t i = 0; i < camera_points.size(); ++i) {
        cam_pts.col(i) << camera_points[i].x, camera_points[i].y, camera_points[i].z;
        obj_pts.col(i) << object_points[i].x, object_points[i].y, object_points[i].z;
    }

    // Compute centroids
    Eigen::Vector3d cam_centroid = cam_pts.rowwise().mean();
    Eigen::Vector3d obj_centroid = obj_pts.rowwise().mean();

    // Center the points
    Eigen::MatrixXd cam_centered = cam_pts.colwise() - cam_centroid;
    Eigen::MatrixXd obj_centered = obj_pts.colwise() - obj_centroid;

    // Compute covariance matrix
    Eigen::MatrixXd covariance = cam_centered * obj_centered.transpose();

    // Perform SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();

    // Compute rotation matrix
    Eigen::Matrix3d rotation = V * U.transpose();
    if (rotation.determinant() < 0) {
        rotation.col(2) *= -1;  // Ensure proper rotation (determinant = 1)
    }

    // Compute translation
    Eigen::Vector3d translation = obj_centroid - rotation * cam_centroid;

    // Construct 4x4 transformation matrix (object to camera)
    Eigen::Matrix4d T_obj_to_cam = Eigen::Matrix4d::Identity();
    T_obj_to_cam.block<3, 3>(0, 0) = rotation;
    T_obj_to_cam.block<3, 1>(0, 3) = translation;

    // Convert to OpenCV Mat
    cv::Mat T_obj_to_cam_cv(4, 4, CV_64F);
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(
            reinterpret_cast<double*>(T_obj_to_cam_cv.data)) = T_obj_to_cam;

    // Set T_world_to_camera
    T_world_to_camera = T_obj_to_cam_cv;

    // Compute transformation to reference frame
    T_world_to_reference = T_camera_to_reference.inv() * T_world_to_camera;

    // Extract rotation and translation
    cv::Mat world_to_reference_rMat = T_world_to_reference(cv::Rect(0, 0, 3, 3));
    cv::Mat world_to_reference_tVec = T_world_to_reference(cv::Rect(3, 0, 1, 3));

    // Compute Euler angles
    cv::Mat mtxR, mtxQ;
    cv::Vec3d euler_angles = cv::RQDecomp3x3(T_world_to_camera(cv::Rect(0, 0, 3, 3)), mtxR, mtxQ);
    cv::Vec3d reference_euler_angles = cv::RQDecomp3x3(world_to_reference_rMat, mtxR, mtxQ);

    // Extract position
    double reference_x = world_to_reference_tVec.at<double>(0, 0);
    double reference_y = world_to_reference_tVec.at<double>(1, 0);
    double reference_z = world_to_reference_tVec.at<double>(2, 0);

    // *** Existing Visualization and Publishing (unchanged) ***
    cv::putText(source_image, "pitch:" + std::to_string(euler_angles[0]), cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(source_image, "yaw:" + std::to_string(euler_angles[1]), cv::Point(20, 110), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(source_image, "roll:" + std::to_string(euler_angles[2]), cv::Point(20, 160), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(source_image, "reference_pitch:" + std::to_string(reference_euler_angles[0]), cv::Point(20, 210), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(source_image, "reference_yaw:" + std::to_string(reference_euler_angles[1]), cv::Point(20, 260), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(source_image, "reference_roll:" + std::to_string(reference_euler_angles[2]), cv::Point(20, 310), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(source_image, "reference_x:" + std::to_string(reference_x), cv::Point(20, 510), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(source_image, "reference_y:" + std::to_string(reference_y), cv::Point(20, 550), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(source_image, "reference_z:" + std::to_string(reference_z), cv::Point(20, 610), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(source_image, "x:" + std::to_string(T_world_to_camera.at<double>(3, 0)), cv::Point(20, 360), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(source_image, "y:" + std::to_string(T_world_to_camera.at<double>(3, 1)), cv::Point(20, 410), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(source_image, "z:" + std::to_string(T_world_to_camera.at<double>(3, 2)), cv::Point(20, 460), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(source_image, "direction:" + ((direction == LEFT_ORIENTATION) ? std::string("LEFT_ORIENTED") : std::string("RIGHT_ORIENTED")), cv::Point(20, 660), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(source_image, "stabilization_state:" + ((stable_frame_count > 20) ? std::string("STABLE") : std::string("UNSTABLE")), cv::Point(20, 710), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::imshow("result", source_image);
    cv::waitKey(1);

    // Stabilization check
    if (abs(reference_x - last_frame_pose.position.x) < 0.5 &&
        abs(reference_y - last_frame_pose.position.y) < 0.5 &&
        abs(reference_z - last_frame_pose.position.z) < 0.5) {
        stable_frame_count++;
    } else {
        stable_frame_count = 0;
    }

    // Publish pose
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "base_link";
    pose_msg.pose.position.x = reference_x;
    pose_msg.pose.position.y = reference_y;
    pose_msg.pose.position.z = reference_z;
    tf2::Quaternion q;
    q.setRPY(-reference_euler_angles[1], reference_euler_angles[2], reference_euler_angles[0]);  // Adjusted order based on your convention
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    msg_interfaces::msg::SlotState current_slot_state;
    current_slot_state.pose = pose_msg;
    current_slot_state.slot_stabled = static_cast<int8_t>(stable_frame_count > 20);
    last_frame_pose = pose_msg.pose;
    slot_state_publisher->publish(current_slot_state);
}
//    if(!sqpnp_initialized) {
//        cv::Mat tVec, rVec;
//        cv::solvePnP(object_points, image_points, CameraMatrix, DistCoeffs, rVec, tVec, false, cv::SOLVEPNP_SQPNP);
//        this->tVec = tVec;
//        this->rVec = rVec;
//        sqpnp_initialized = true;
//    }
//    else {
//        cv::solvePnP(object_points, image_points, CameraMatrix, DistCoeffs, this->rVec, this->tVec, true, cv::SOLVEPNP_ITERATIVE);
//        cv::Mat rotation_matrix;
//        cv::Rodrigues(this->rVec, rotation_matrix);
//        rotation_matrix.copyTo(T_world_to_camera(cv::Rect(0, 0, 3, 3)));
//        this->tVec.copyTo(T_world_to_camera(cv::Rect(3, 0, 1, 3)));
//        T_world_to_reference = T_camera_to_reference.inv() * T_world_to_camera;
//        cv::Mat world_to_reference_rMat = T_world_to_reference(cv::Rect(0, 0, 3, 3));
//        cv::Mat world_to_reference_tVec = T_world_to_reference(cv::Rect(3, 0, 1, 3));
//        cv::Mat mtxR, mtxQ;
//        cv::Vec3d euler_angles = cv::RQDecomp3x3(rotation_matrix, mtxR, mtxQ, cv::noArray(),cv::noArray());
//        if (euler_angles[0] > 0 || abs(euler_angles[0]) > 90 || abs(euler_angles[1]) > 135 ) {
//            sqpnp_initialized = false;
//            return;
//        }
//        cv::Vec3d reference_euler_angles = cv::RQDecomp3x3(world_to_reference_rMat, mtxR, mtxQ, cv::noArray(),cv::noArray());
//        double reference_x = world_to_reference_tVec.at<double>(0, 0);
//        double reference_y = world_to_reference_tVec.at<double>(1, 0);
//        double reference_z = world_to_reference_tVec.at<double>(2, 0);
//        double reference_pitch = reference_euler_angles[0];
//        double reference_yaw = reference_euler_angles[1];
//        double reference_roll = reference_euler_angles[2];
//        //print euler angle on the screen through imshow
//        cv::putText(source_image, "pitch:" + std::to_string(euler_angles[0]), cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
//        cv::putText(source_image, "yaw:" + std::to_string(euler_angles[1]), cv::Point(20, 110), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
//        cv::putText(source_image, "roll:" + std::to_string(euler_angles[2]), cv::Point(20, 160), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
//        cv::putText(source_image, "reference_pitch:" + std::to_string(reference_euler_angles[0]), cv::Point(20, 210), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
//        cv::putText(source_image, "reference_yaw:" + std::to_string(reference_euler_angles[1]), cv::Point(20, 260), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
//        cv::putText(source_image, "reference_roll:" + std::to_string(reference_euler_angles[2]), cv::Point(20, 310), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
//        cv::putText(source_image, "reference_x:" + std::to_string(world_to_reference_tVec.at<double>(0, 0)), cv::Point(20, 510), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
//        cv::putText(source_image, "reference_y:" + std::to_string(world_to_reference_tVec.at<double>(1, 0)), cv::Point(20, 550), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
//        cv::putText(source_image, "reference_z:" + std::to_string(world_to_reference_tVec.at<double>(2, 0)), cv::Point(20, 610), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
//        cv::putText(source_image, "x:" + std::to_string(this->tVec.at<double>(0, 0)), cv::Point(20, 360), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
//        cv::putText(source_image, "y:" + std::to_string(this->tVec.at<double>(1, 0)), cv::Point(20, 410), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
//        cv::putText(source_image, "z:" + std::to_string(this->tVec.at<double>(2, 0)), cv::Point(20, 460), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
//        cv::putText(source_image, "direction:" + ((direction == LEFT_ORIENTATION) ? std::string("LEFT_ORIENTED") : std::string("RIGHT_ORIENTED")), cv::Point(20, 660), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
//        cv::putText(source_image, "stabilization_state:" + ((stable_frame_count > 20) ? std::string("STABLE") : std::string("UNSTABLE")), cv::Point(20, 710), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
//        cv::imshow("result", source_image);
//        cv::waitKey(1);
//        //wait until the slot stops moving
//        if(abs(reference_x - last_frame_pose.position.x ) < 0.5
//           && abs(reference_y - last_frame_pose.position.y) < 0.5
//           && abs(reference_z - last_frame_pose.position.z) < 0.5
//                ) {
//            stable_frame_count++;
//        } else {
//            stable_frame_count = 0;
//        }
//        geometry_msgs::msg::PoseStamped pose_msg;
//        pose_msg.header.stamp = this->now();
//        pose_msg.header.frame_id = "base_link";
//        pose_msg.pose.position.x = world_to_reference_tVec.at<double>(0, 0);
//        pose_msg.pose.position.y = world_to_reference_tVec.at<double>(1, 0);
//        pose_msg.pose.position.z = world_to_reference_tVec.at<double>(2, 0);
//        tf2::Quaternion q;
//        q.setRPY(-reference_yaw, reference_roll, reference_pitch);
//        pose_msg.pose.orientation.x = q.x();
//        pose_msg.pose.orientation.y = q.y();
//        pose_msg.pose.orientation.z = q.z();
//        pose_msg.pose.orientation.w = q.w();
//        msg_interfaces::msg::SlotState current_slot_state;
//        current_slot_state.pose = pose_msg;
//        current_slot_state.slot_stabled = static_cast<int8_t >(stable_frame_count > 20);
//        last_frame_pose = pose_msg.pose;
//        slot_state_publisher->publish(current_slot_state);
//    }
//}

float side_sign_detector::get_depth(int u, int v) {
    // Check if the depth image has been received
    if (depth_image.empty()) {
        RCLCPP_WARN(this->get_logger(), "Depth image not yet received");
        return -1.0f;  // Return invalid depth
    }

    // Validate pixel coordinates
    if (u < 0 || u >= depth_image.cols || v < 0 || v >= depth_image.rows) {
        RCLCPP_WARN(this->get_logger(), "Pixel (%d, %d) out of bounds", u, v);
        return -1.0f;  // Return invalid depth
    }

    // Retrieve depth value (row = v, col = u in OpenCV)
    float depth = depth_image.at<float>(v, u);

    // Check for invalid depth (NaN or non-positive)
    if (std::isnan(depth) || depth <= 0.0f) {
        RCLCPP_WARN(this->get_logger(), "Invalid depth at (%d, %d)", u, v);
        return -1.0f;  // Return invalid depth
    }

    return depth;  // Return valid depth in meters
}

cv::Point3f side_sign_detector::deproject_pixel_to_point(int u, int v, float depth) {
    double fx = D435i_CameraMatrix.at<double>(0, 0);
    double fy = D435i_CameraMatrix.at<double>(1, 1);
    double cx = D435i_CameraMatrix.at<double>(0, 2);
    double cy = D435i_CameraMatrix.at<double>(1, 2);

    float x = (u - cx) * depth / fx;
    float y = (v - cy) * depth / fy;
    return cv::Point3f(x, y, depth);
}

RCLCPP_COMPONENTS_REGISTER_NODE(
        side_sign_detector
        )