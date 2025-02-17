#include "../include/front_sign_detector/front_sign_detector.hpp"
#include <rclcpp_components/register_node_macro.hpp>

front_sign_detector::front_sign_detector(const rclcpp::NodeOptions &options) : Node("front_sign_detector", options) {
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
    lastFrameCorners.push_back(cv::Point(100, 100));
    lastFrameCorners.emplace_back(cv::Point(100, -100));
    lastFrameCorners.emplace_back(cv::Point(-100, -100));
    lastFrameCorners.emplace_back(cv::Point(-100, 100));
    currentFrameCorners = lastFrameCorners;
    sucker_goal_publisher = this->create_publisher<msg_interfaces::msg::SlotState>("sucker_goal", rclcpp::QoS(10));
    slot_state_publisher = this->create_publisher<msg_interfaces::msg::SlotState>("slot_state", rclcpp::QoS(10));
    image_publisher = this->create_publisher<sensor_msgs::msg::Image>("processed_image", rclcpp::QoS(10));
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

void front_sign_detector::imageCallback(const sensor_msgs::msg::Image::SharedPtr &msg) {
    //calculate time elapse
//    auto current_time = this->now();
//    auto time_elapse = (current_time - msg->header.stamp).seconds() * 1000;
//    RCLCPP_INFO(this->get_logger(), "Time elapse: %f ms", time_elapse);
    auto img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    processImage(img);
    selectContours();
    getCorners();
    solveAngle();
}

void front_sign_detector::processImage(const cv::Mat &image) {
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

void front_sign_detector::selectContours() {
    found = false; //Reset state
    currentFrameSmallSquares.clear();
	double minArea = 1000, maxArea = 10000; // For a single contour
	double maxRatio = 4.5; // For a single contour : width / height
    //double minDis = 0, maxDis = 400; // Compare between contours
    double minAreaRatio = 0.2, maxAreaRatio = 5; // Compare between contours
    double minSmallSquareArea = 500, maxSmallSquareArea = 2000;
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(processed_image, contours, cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);// should be in counter clock-wise order according to the video
    std::vector<candidateContour> candidateContours; // Potential corner contours

//    for (auto& candidate : contours) {
//        std::vector<std::vector<cv::Point>> contour = { candidate };
//        cv::drawContours(source, contour, 0, cv::Scalar(255, 0, 0), 2);
//        cv::imshow("Polygons", source);
//        cv::waitKey(500);
//    }
//
	for (const auto& contour : contours) {
		double area = cv::contourArea(contour);
        std::vector<std::vector<cv::Point>> contour_ = { contour };
        cv::drawContours(source_image, contour_, 0, cv::Scalar(0, 255, 0), 2);

        cv::Moments moments = cv::moments(contour);
        int centerX = static_cast<int>(moments.m10 / moments.m00);
        int centerY = static_cast<int>(moments.m01 / moments.m00);

        std::string areaText = "Area: " + std::to_string(area);
        cv::putText(source_image, areaText, cv::Point(centerX, centerY), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

        cv::RotatedRect boundary = cv::minAreaRect(contour);
        //select two small Points
        double boundArea = boundary.size.height * boundary.size.width;
        if (area > minSmallSquareArea && area < maxSmallSquareArea && boundArea < 2 * area && boundary.size.width / boundary.size.height < 2 && boundary.size.height / boundary.size.width < 2) {
            if (currentFrameSmallSquares.size() < 2)
                currentFrameSmallSquares.push_back(boundary.center);
            continue;
        }
		//Area Judge
		if (area < minArea || area > maxArea)
			continue;
		//approxPolygon Judge
        std::vector<cv::Point> approx;
        double epsilon = 0.02 * cv::arcLength(contour, true);
        cv::approxPolyDP(contour, approx, epsilon, true);
        //cv::polylines(source, approx, true, cv::Scalar(255, 0, 0), 10);
        if (approx.size() <= 5 || approx.size() >= 13) {
            continue;
        }

        //Ratio Judge
		double widthHeightRatio = static_cast<double>(boundary.size.width) / boundary.size.height;
        double heightWidthRatio = static_cast<double>(boundary.size.height) / boundary.size.width;
		if (widthHeightRatio > maxRatio || heightWidthRatio > maxRatio)
			continue;
        candidateContour chosenContour(contour);
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);
        chosenContour.corner = center;
        chosenContour.area = cv::contourArea(contour);
		candidateContours.push_back(chosenContour);
	}

    if (!currentFrameSmallSquares.empty())
        lastFrameSmallSquares = currentFrameSmallSquares;

    //std::cout << "currentFrameSmallSquares: " << currentFrameSmallSquares.size() << std::endl;

    if (candidateContours.size() < 4) {
        return;
    }

    cv::Point2f center = computeCentroid(candidateContours);
    this->center = center;
    std::sort(candidateContours.begin(), candidateContours.end(), [&](const candidateContour& a, const candidateContour& b) {
        return compareByRelativeAngle(a, b, center);
        });
    /*
    cv::Mat Source = source.clone();
    for (auto& candidate : candidateContours) {
        std::vector<std::vector<cv::Point>> contour = { candidate.contour };
        cv::drawContours(Source, contour, -1, cv::Scalar(255, 0, 0), 2);
        cv::imshow("Polygons", Source);
        cv::waitKey(500);
    }
    */
    int contourNum = candidateContours.size();
    //Find corner set
    UnionFind UF(contourNum);
    for (int i = 0; i < contourNum; i += 1) {
        for (int j = i + 1; j < contourNum; j += 1) {
            //double distance = cv::norm(candidateContours[i].corner - candidateContours[j].corner);
            double areaRatio = candidateContours[i].area / candidateContours[j].area;
            if (areaRatio > minAreaRatio && areaRatio < maxAreaRatio)
                UF.unite(i, j);
        }
    }
    // Find a set with 4 contours
    std::vector<int> setSizes(contourNum, 0);
    for (int i = 0; i < contourNum; i++) {
        setSizes[UF.find(i)]++;
    }
    int root = -1;
    for (int i = 0; i < contourNum; i++) {
        if (setSizes[i] == 4) {
            root = i;
            break;
        }
    }
    if (root == -1) {
        //std::cerr << "No suitable Area Found!" << std::endl;
        return;
    }
    found = true;
    std::vector<candidateContour> resultCorners;
    for (int i = 0; i < contourNum; i++) {
        if (UF.same(i, root)) {
            resultCorners.push_back(candidateContours[i]);
        }
    }
    this->corners = resultCorners;
}

void front_sign_detector::getCorners() {
    if (found == false)
        return;
    //std::cout << "Found suitable area" << std::endl;
    cv::circle(source_image, center, 10, cv::Scalar(0, 255, 0), -1);
    for (auto& corner : this->corners) {
        std::vector<cv::Point> approxTriangle;
        cv::minEnclosingTriangle(corner.contour, approxTriangle);
        //cv::polylines(source, approxTriangle, true, cv::Scalar(255, 0, 0), 10);
        /*
        std::vector<std::vector<cv::Point>> contours = { approxTriangle };
        cv::Mat Source = source.clone();
        cv::drawContours(Source, contours, 0, cv::Scalar(255, 0, 0), 2);
        cv::imshow("approxResult", Source);
        cv::waitKey(500);
        */
        double maxAngle = 0;
        cv::Point maxAnglePoint;
        for (int i = 0; i < 3; i += 1) {
            // iterate through 3 points and calculate the angles
            cv::Point a = approxTriangle[i];
            cv::Point b = approxTriangle[(i + 1) % 3];
            cv::Point c = approxTriangle[(i + 2) % 3];

            cv::Point ab = { a.x - b.x, a.y - b.y };
            cv::Point cb = { c.x - b.x, c.y - b.y };
            // tan�� = crossProduct / dotProduct
            double dotProduct = ab.x * cb.x + ab.y * cb.y;
            double crossProduct = ab.x * cb.y - ab.y * cb.x;
            double angle = std::atan(std::abs(crossProduct) / std::abs(dotProduct));
            if (angle > maxAngle) {
                maxAngle = angle;
                maxAnglePoint = b;
            }
        }
        corner.corner = maxAnglePoint;
        if (maxAngle * (180 / M_PI) > 130)
            corner.corner = maxAnglePoint;
        else {
            double maxDistance = 0;
            int maxDisIndex;
            for (int i = 0; i < 3; i++) {
                double distance = cv::norm(static_cast<cv::Point2f>(approxTriangle[i]) - this->center);
                if (distance > maxDistance) {
                    maxDistance = distance;
                    maxDisIndex = i;
                }
            }
            corner.corner = approxTriangle[maxDisIndex];
        }
        //cv::circle(source, corner.corner, 10, cv::Scalar(0, 255, 0), -1);

    }
    for (const auto& point : currentFrameSmallSquares) {
        cv::circle(source_image, point, 10, cv::Scalar(255, 0, 0), -1);
    }
    //cv::imshow("cornerPoints", source);

    // Use area to judge which one is the up right corner
    if (currentFrameSmallSquares.empty())
        currentFrameSmallSquares = lastFrameSmallSquares;
    double minDis = 1000000;
    auto minAreaContour = this->corners.begin();
        for (auto iterator = corners.begin(); iterator < corners.end(); iterator++) {
            double distanceFromSquares = 0;
            for (auto& point : currentFrameSmallSquares) {
                distanceFromSquares += cv::norm(point - iterator->corner);
            }
            if (distanceFromSquares < minDis) {
                minDis = distanceFromSquares;
                minAreaContour = iterator;
            }
        }

    cv::circle(source_image, minAreaContour->corner, 10, cv::Scalar(0, 255, 0), -1);
    std::rotate(this->corners.begin(), minAreaContour, this->corners.end());

    for (int i = 0; i < 4; i += 1) {
        currentFrameCorners[i] = 0.4 * corners[i].corner + 0.6 * static_cast<cv::Point2f>(lastFrameCorners[i]);
    }
    //cv::polylines(source_image, currentFrameCorners, true, cv::Scalar(255, 0, 0), 10);
    for (int i = 0; i < 4; i += 1) {
        cv::circle(source_image, corners[i].corner, 5, cv::Scalar(255, 0, 0), -1);
    }
}

void front_sign_detector::solveAngle() {
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

    float size = 288.0;
    std::vector<cv::Point3f> object_points = { cv::Point3f(size / 2, -size / 2,-size / 2),
                                          cv::Point3f(size / 2, size / 2,-size / 2),
                                          cv::Point3f(-size / 2, size / 2,-size / 2),
                                          cv::Point3f(-size / 2, -size / 2,-size / 2) };
    //for computing the position and orientation of the desired sucker goal state
    std::vector<cv::Point3f> object_points_ = { cv::Point3f(size / 2, -size / 2, size / 2),
                                          cv::Point3f(size / 2, size / 2, size / 2),
                                          cv::Point3f(-size / 2, size / 2, size / 2),
                                          cv::Point3f(-size / 2, -size / 2, size / 2) };
    std::vector<cv::Point2f> image_points = { this->corners[0].corner,
                                     this->corners[1].corner,
                                     this->corners[2].corner,
                                     this->corners[3].corner };

    cv::solvePnP(object_points, image_points, CameraMatrix, DistCoeffs, this->rVec, this->tVec, false, cv::SOLVEPNP_SQPNP);
    cv::Mat rotation_matrix;
    cv::Rodrigues(this->rVec, rotation_matrix);
    rotation_matrix.copyTo(T_world_to_camera(cv::Rect(0, 0, 3, 3)));
    this->tVec.copyTo(T_world_to_camera(cv::Rect(3, 0, 1, 3)));
    T_world_to_reference = T_camera_to_reference.inv() * T_world_to_camera;
    cv::Mat world_to_reference_rMat = T_world_to_reference(cv::Rect(0, 0, 3, 3));
    cv::Mat world_to_reference_tVec = T_world_to_reference(cv::Rect(3, 0, 1, 3));
    cv::Mat mtxR, mtxQ;
    cv::Vec3d euler_angles = cv::RQDecomp3x3(rotation_matrix, mtxR, mtxQ, cv::noArray(),cv::noArray());
    cv::Vec3d reference_euler_angles = cv::RQDecomp3x3(world_to_reference_rMat, mtxR, mtxQ, cv::noArray(),cv::noArray());
    double reference_x = world_to_reference_tVec.at<double>(2, 0);
    double reference_y = - world_to_reference_tVec.at<double>(0, 0);
    double reference_z = - world_to_reference_tVec.at<double>(1, 0);
    double reference_pitch = reference_euler_angles[0];
    double reference_yaw = reference_euler_angles[1];
    double reference_roll = reference_euler_angles[2];

    cv::Mat rVecSucker, tVecSucker;
    cv::solvePnP(object_points_, image_points, CameraMatrix, DistCoeffs, rVecSucker, tVecSucker, false, cv::SOLVEPNP_SQPNP);
    cv::Mat rotation_matrix_;
    cv::Rodrigues(rVecSucker, rotation_matrix_);
    cv::Mat T_world_to_camera_ = cv::Mat::eye(4, 4, CV_64F);
    rotation_matrix_.copyTo(T_world_to_camera_(cv::Rect(0, 0, 3, 3)));
    tVecSucker.copyTo(T_world_to_camera_(cv::Rect(3, 0, 1, 3)));
    cv::Mat T_world_to_reference_ = cv::Mat::eye(4, 4, CV_64F);
    T_world_to_reference_ = T_camera_to_reference.inv() * T_world_to_camera_;
    cv::Mat world_to_reference_rMat_ = T_world_to_reference_(cv::Rect(0, 0, 3, 3));
    cv::Mat world_to_reference_tVec_ = T_world_to_reference_(cv::Rect(3, 0, 1, 3));
    cv::Mat mtxR_, mtxQ_;
    cv::Vec3d reference_euler_angles_ = cv::RQDecomp3x3(world_to_reference_rMat_, mtxR_, mtxQ_, cv::noArray(),cv::noArray());
    double reference_x_ = world_to_reference_tVec_.at<double>(2, 0);
    double reference_y_ =  - world_to_reference_tVec_.at<double>(0, 0);
    double reference_z_ =  - world_to_reference_tVec_.at<double>(1, 0);
    double reference_pitch_ = reference_euler_angles_[0];
    double reference_yaw_ = reference_euler_angles_[1];
    double reference_roll_ = reference_euler_angles_[2];




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
    cv::putText(source_image, "stabilization_state:" + ((stable_frame_count > 20) ? std::string("STABLE") : std::string("UNSTABLE")), cv::Point(20, 710), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    //cv::imshow("result", source_image);
    //cv::waitKey(1);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "base_link";
    pose_msg.pose.position.x = reference_x / 1000;
    pose_msg.pose.position.y = reference_y / 1000;
    pose_msg.pose.position.z = reference_z / 1000;
    tf2::Quaternion q;
    q.setRPY(-reference_roll / 180 * M_PI, -reference_pitch / 180 * M_PI, -reference_yaw / 180 * M_PI);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();
    if(     abs(last_frame_pose.position.x - pose_msg.pose.position.x) < 0.01 &&
            abs(last_frame_pose.position.y - pose_msg.pose.position.y) < 0.01 &&
            abs(last_frame_pose.position.z - pose_msg.pose.position.z) < 0.01
            ) {
        stable_frame_count += 1;
    } else {
        stable_frame_count = 0;
    }
    //publisher slot state
    msg_interfaces::msg::SlotState current_slot_state;
    current_slot_state.pose = pose_msg;
    current_slot_state.slot_stabled = static_cast<int8_t >(stable_frame_count > 50);
    last_frame_pose = pose_msg.pose;
    slot_state_publisher->publish(current_slot_state);
    //publish sucker goal
    msg_interfaces::msg::SlotState sucker_goal;
    sucker_goal.pose.header.stamp = this->now();
    sucker_goal.pose.header.frame_id = "base_link";
    sucker_goal.slot_stabled = static_cast<int8_t >(stable_frame_count > 50);
    sucker_goal.pose.pose.position.x = reference_x_ / 1000;
    sucker_goal.pose.pose.position.y = - reference_y_ / 1000;
    sucker_goal.pose.pose.position.z = - reference_z_ / 1000;
    tf2::Quaternion q_;
    q_.setRPY(-reference_roll_ / 180 * M_PI,  M_PI + reference_pitch_ / 180 * M_PI, M_PI - reference_yaw_ / 180 * M_PI);
    sucker_goal.pose.pose.orientation.x = q_.x();
    sucker_goal.pose.pose.orientation.y = q_.y();
    sucker_goal.pose.pose.orientation.z = q_.z();
    sucker_goal.pose.pose.orientation.w = q_.w();

    sucker_goal_publisher->publish(sucker_goal);


    //publish the processed source image
    sensor_msgs::msg::Image::SharedPtr processed_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", source_image).toImageMsg();
    image_publisher->publish(*processed_image_msg);
}

cv::Point2f front_sign_detector::computeCentroid(const std::vector<candidateContour>& candidates) {
    cv::Point2f center(0, 0);
    for (const candidateContour& candidate : candidates) {
        center.x += candidate.corner.x;
        center.y += candidate.corner.y;
    }
    center.x /= candidates.size();
    center.y /= candidates.size();
    return center;
}


bool front_sign_detector::compareByRelativeAngle(const candidateContour& a, const candidateContour& b, const cv::Point2f& center) {
    float angleA = std::atan2(a.corner.y - center.y, a.corner.x - center.x);
    float angleB = std::atan2(b.corner.y - center.y, b.corner.x - center.x);
    return angleA < angleB;
}



RCLCPP_COMPONENTS_REGISTER_NODE(
        front_sign_detector
)