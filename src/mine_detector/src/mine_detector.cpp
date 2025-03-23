#include <mine_detector/mine_detector.hpp>

mine_detector::mine_detector(const rclcpp::NodeOptions &options) : Node("mine_detector", options) {
    loadModel();
    image_subscription = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera1/image_raw",
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                imageCallback(msg);
            });

}

void mine_detector::loadModel() {
    try {
        // Load the ONNX model
        model = cv::dnn::readNetFromONNX("/home/engineer/code/rm_eng/src/mine_detector/best.onnx");
        if (model.empty()) {
            std::cerr << "Error: Could not load the ONNX model!" << std::endl;
        }
        // Add class name
        classes.push_back("mine");

        RCLCPP_INFO(this->get_logger(), "Model loaded successfully");
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error loading model: %s", e.what());
    }
}

std::vector<cv::Rect> mine_detector::detectROI(const cv::Mat &image) {
    std::vector<cv::Rect> detections;

    try {
        // Create blob from image (same as in test.py with 640x640 size)
        cv::Mat blob = cv::dnn::blobFromImage(
                image, 1/255.0, cv::Size(640, 640),
                cv::Scalar(), true, false // true for swapRB (BGR to RGB)
        );

        // Set the input
        model.setInput(blob);

        // Run forward pass (similar to results = model.predict())
        std::vector<cv::Mat> outputs;
        model.forward(outputs, model.getUnconnectedOutLayersNames());

        // Process detections - similar to how test.py processes results
        std::vector<int> classIds;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;

        // The ONNX model likely has a different output format than the raw PyTorch model
        // Check the first output shape to determine format
        const cv::Mat& output = outputs[0];

        // YOLOv8 ONNX output is typically [1, num_classes+5, num_boxes] or [1, num_boxes, num_classes+5]
        // We need to determine the exact format based on the output shape
        if (!output.empty()) {
            // Get output dimensions
            const int rows = output.rows;
            const int dimensions = output.cols;

            RCLCPP_INFO(this->get_logger(), "Output shape: %d x %d", rows, dimensions);

            // Process based on typical YOLOv8 ONNX output format
            for (int i = 0; i < rows; i++) {
                float* row_ptr = (float*)output.row(i).data;

                // First 4 values are box coordinates (x, y, w, h), 5th is confidence
                float x = row_ptr[0];
                float y = row_ptr[1];
                float w = row_ptr[2];
                float h = row_ptr[3];
                float conf = row_ptr[4];

                if (conf >= confThreshold) {
                    // Find class with highest score (starts at index 5)
                    float max_class_score = 0;
                    int max_class_id = 0;

                    for (int c = 0; c < classes.size(); c++) {
                        float class_score = row_ptr[5 + c];
                        if (class_score > max_class_score) {
                            max_class_score = class_score;
                            max_class_id = c;
                        }
                    }

                    // Same conversion as in test.py
                    int centerX = static_cast<int>(x * image.cols);
                    int centerY = static_cast<int>(y * image.rows);
                    int width = static_cast<int>(w * image.cols);
                    int height = static_cast<int>(h * image.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;

                    classIds.push_back(max_class_id);
                    confidences.push_back(conf);
                    boxes.push_back(cv::Rect(left, top, width, height));

                    RCLCPP_INFO(this->get_logger(), "Detected: %s, Confidence: %.3f, Box: [%d, %d, %d, %d]",
                                classes[max_class_id].c_str(), conf, left, top, left+width, top+height);
                }
            }
        }

        // Apply non-maximum suppression (same as in test.py)
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);

        for (size_t i = 0; i < indices.size(); ++i) {
            int idx = indices[i];
            detections.push_back(boxes[idx]);
        }

    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Detection error: %s", e.what());
    }

    return detections;
}

// Change function signature
void mine_detector::processROI(cv::Mat &image, const cv::Rect &roi) {
    try {
        if(roi.x < 0 || roi.y < 0 || roi.width < 0 || roi.height < 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid ROI");
            return;
        }
        // Extract the ROI
        cv::Mat roiMat = image(roi);

        // Convert to grayscale
        cv::Mat gray;
        cv::cvtColor(roiMat, gray, cv::COLOR_BGR2GRAY);

        // Apply Gaussian blur
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

        // Apply adaptive thresholding
        cv::Mat binary;
        cv::adaptiveThreshold(gray, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                              cv::THRESH_BINARY_INV, 11, 2);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Draw contours on the original ROI
        cv::drawContours(roiMat, contours, -1, cv::Scalar(0, 255, 0), 2);

        // Draw the ROI rectangle on the original frame
        cv::rectangle(image, roi, cv::Scalar(255, 0, 0), 2);
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "ROI processing error: %s", e.what());
    }
}

cv::Mat mine_detector::preProcessImage(const cv::Mat& image, const cv::Size& size) {
    // Convert to grayscale
    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

    // Resize image
    cv::Mat resized_image;
    cv::resize(gray_image, resized_image, size);

    // Normalize pixel values
    cv::Mat normalized_image;
    resized_image.convertTo(normalized_image, CV_32F, 1.0/255.0);

    // Convert back to 8-bit for display purposes
    cv::Mat output_image;
    normalized_image.convertTo(output_image, CV_8U, 255.0);

    // Convert back to 3 channels for the detector
    cv::Mat output_color;
    cv::cvtColor(output_image, output_color, cv::COLOR_GRAY2BGR);

    return output_color;
}

void mine_detector::imageCallback(const sensor_msgs::msg::Image::SharedPtr &msg) {
    try {
        // Convert ROS Image to OpenCV Image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame = cv_ptr->image;
        cv::Mat preprocessed_frame = preProcessImage(frame);
        // Detect ROIs using YOLO
        std::vector<cv::Rect> rois = detectROI(preprocessed_frame);

        // Process each ROI
        for (const auto& roi : rois) {
            processROI(preprocessed_frame, roi);
        }
        cv::imshow("Frame", preprocessed_frame);
        cv::waitKey(1);

    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV error: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
    }
}


RCLCPP_COMPONENTS_REGISTER_NODE(
        mine_detector
)