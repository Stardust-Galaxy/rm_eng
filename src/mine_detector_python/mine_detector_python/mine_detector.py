#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from ultralytics import YOLO
import os
import sys
import math
class MineDetectorNode(Node):
    def __init__(self):
        super().__init__('mine_detector_python')

        # Create CV bridge
        self.bridge = CvBridge()

        # Store source image
        self.source_image = None

        # Configure YOLO model
        self.model_path = os.path.expanduser('~/code/rm_eng/src/mine_detector/best.pt')
        self.conf_threshold = 0.5
        self.nms_threshold = 0.4

        # Load the model
        self.load_model()

        # Create subscription to camera topic
        self.image_subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('Mine Detector Node initialized')

    def load_model(self):
        """Load the YOLO model"""
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f'Model loaded successfully: {self.model_path}')
        except Exception as e:
            self.get_logger().error(f'Error loading model: {e}')
            sys.exit(1)

    def pre_process_image(self, image, size=(640, 480)):
        """Preprocess the image before detection"""
        # Convert to grayscale
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Enhance contrast
        # equalized_image = cv2.equalizeHist(gray_image)

        # Resize image
        resized_image = cv2.resize(gray_image, size)

        # Normalize pixel values
        normalized_image = resized_image / 255.0

        # Convert back to 8-bit for display
        output_image = (normalized_image * 255).astype(np.uint8)

        # Convert back to 3 channels
        output_color = cv2.cvtColor(output_image, cv2.COLOR_GRAY2BGR)

        return output_color

    def detect_roi(self, image):
        """Detect regions of interest using YOLO following test.py implementation"""
        detections = []

        # Run inference - similar to test.py
        results = self.model(
            source=image,
            conf=self.conf_threshold,
            imgsz=640,  # Match the size used in test.py
        )

        # Process results
        result = results[0]  # First result
        boxes = result.boxes  # Bounding boxes

        # Process each box like in test.py
        for box in boxes:
            # Extract details
            cls_id = int(box.cls)  # Class ID
            label = self.model.names[cls_id]  # Class name
            conf = box.conf.item()  # Confidence score

            # Get coordinates in xyxy format (as in test.py)
            x_min, y_min, x_max, y_max = map(int, box.xyxy[0])

            # Draw rectangle and text (just like in test.py)
            cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

            # Add label and confidence above box
            text = f"{label} {conf:.2f}"
            cv2.putText(
                image, text, (x_min, y_min - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2
            )

            # For compatibility with process_roi, convert to xywh format
            w = x_max - x_min
            h = y_max - y_min

            # Log detection details like in test.py
            self.get_logger().info(f"Detected: {label}, Confidence: {conf:.3f}, Box: [{x_min}, {y_min}, {x_max}, {y_max}]")

            # Add to detections list for further processing
            detections.append((x_min, y_min, w, h))

        return detections



    def find_contours_and_corners(self, binary_image, roi_mat, roi_x, roi_y):
        """
        Find rectangular contours and use their corners directly
        """
        # Parameters for contour selection
        min_area = 1000.0
        max_area = 5000.0
        max_ratio = 1.5

        # Find contours
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Create debug image for visualization
        debug_image = self.source_image.copy()

        # Store rectangle corners directly
        all_corners = []
        rectangle_centers = []

        # Process each contour
        for contour in contours:
            area = cv2.contourArea(contour)

            # Skip based on area
            if area < min_area or area > max_area:
                continue

            # Get minimum area rectangle
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # Convert to global coordinates
            global_box = box + np.array([roi_x, roi_y])

            # Get width and height
            width = rect[1][0]
            height = rect[1][1]

            # Skip based on aspect ratio
            if max(width, height) / min(width, height) > max_ratio:
                continue

            # Calculate rectangle center in global coordinates
            center_x = rect[0][0] + roi_x
            center_y = rect[0][1] + roi_y

            # Draw the rectangle
            cv2.drawContours(debug_image, [global_box], 0, (0, 0, 255), 2)
            cv2.putText(debug_image, f"Area: {int(area)}", (int(center_x), int(center_y)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

            # Store rectangle center
            rectangle_centers.append((center_x, center_y))

            # Store all corners from this rectangle
            for corner in global_box:
                all_corners.append(tuple(corner))

        # If no rectangles found
        if len(rectangle_centers) == 0:
            cv2.imshow("Processed Source", debug_image)
            cv2.waitKey(1)
            return []

        # Calculate centroid of all rectangle centers
        center_x = sum(c[0] for c in rectangle_centers) / len(rectangle_centers)
        center_y = sum(c[1] for c in rectangle_centers) / len(rectangle_centers)
        center = (int(center_x), int(center_y))

        # Draw centroid
        cv2.circle(debug_image, center, 10, (0, 255, 0), -1)

        # Sort corners by distance from center to get outer corners
        corners_with_distance = [(corner, math.sqrt((corner[0] - center_x)**2 + (corner[1] - center_y)**2))
                                 for corner in all_corners]
        corners_with_distance.sort(key=lambda x: x[1], reverse=True)

        # Take the furthest corners (up to 4)
        outer_corners = []
        for corner, _ in corners_with_distance[:min(4, len(corners_with_distance))]:
            if corner not in outer_corners:  # Avoid duplicates
                outer_corners.append(corner)

        # Sort by angle from center for consistent ordering
        outer_corners.sort(key=lambda c: math.atan2(c[1] - center_y, c[0] - center_x))

        # Draw outer corners
        for corner in outer_corners:
            cv2.circle(debug_image, corner, 5, (255, 0, 0), -1)

        cv2.imshow("Processed Source", debug_image)
        cv2.waitKey(1)

        return outer_corners

    def process_roi(self, roi):
        """Process a region of interest"""
        x, y, w, h = roi

        # Validate ROI coordinates
        if x < 0 or y < 0 or w < 0 or h < 0:
            self.get_logger().error("Invalid ROI")
            return

        # Ensure ROI does not extend beyond image boundaries
        if x + w > self.source_image.shape[1] or y + h > self.source_image.shape[0]:
            self.get_logger().error("ROI outside image boundaries")
            return

        try:
            # Extract ROI for contour detection only
            roi_mat = self.source_image[y:y+h, x:x+w]

            # Convert to grayscale
            gray = cv2.cvtColor(roi_mat, cv2.COLOR_BGR2GRAY)

            # Apply Gaussian blur
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)

            # Apply histogram equalization for dark conditions
            equalized = cv2.equalizeHist(blurred)

            # Apply fixed threshold with BINARY_INV for dark conditions
            _, binary = cv2.threshold(equalized, 65, 255, cv2.THRESH_BINARY_INV)

            # Find contours in ROI but translate to global coordinates
            corners = self.find_contours_and_corners(binary, roi_mat, x, y)

            # Draw rectangle around ROI on source image
            cv2.rectangle(self.source_image, (x, y), (x+w, y+h), (255, 0, 0), 2)

            # Display the binary image from ROI processing
            cv2.imshow("Binary ROI", binary)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"ROI processing error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())


    def image_callback(self, msg):
        """Callback when a new image is received"""
        try:
            # Convert ROS Image to OpenCV Image
            self.source_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Create a working copy for display
            display_image = self.source_image.copy()

            # Detect ROIs using YOLO
            rois = self.detect_roi(display_image)

            # Process each ROI
            for roi in rois:
                self.process_roi(roi)

            # Display the image
            cv2.imshow("Mine Detection", display_image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MineDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()