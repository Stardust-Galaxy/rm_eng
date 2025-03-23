from ultralytics import YOLO
import cv2
import os

# Define paths
MODEL_PATH = r'D:\Pycharm Projects\mine_detection\runs\detect\train2\weights\best.pt'  # Path to your trained model
INPUT_IMAGE = r'D:\Pycharm Projects\mine_detection\preprocessed_test.jpg'  # Path to your single test image
OUTPUT_IMAGE = r'D:\Pycharm Projects\mine_detection\predicted_image.jpg'  # Where to save the result


def predict_and_draw(model_path, input_image, output_image, conf_threshold=0.5, imgsz=640, show_image=False):
    """
    Predict objects in a single image using a YOLOv8 model and draw bounding boxes.

    Args:
        model_path (str): Path to the trained model weights (.pt file).
        input_image (str): Path to the input image.
        output_image (str): Path to save the output image with bounding boxes.
        conf_threshold (float): Confidence threshold for detections (0-1).
        imgsz (int): Image size for inference (matches training size).
        show_image (bool): If True, display the result using OpenCV.
    """
    # Load the model
    model = YOLO(model_path)

    # Check if input image exists
    if not os.path.exists(input_image):
        raise FileNotFoundError(f"Input image not found at: {input_image}")
    # Run inference
    results = model.predict(
        source=input_image,
        conf=conf_threshold,  # Confidence threshold
        imgsz=imgsz,  # Image size (640x640 from training)
        device=0  # Use GPU (set to 'cpu' if no GPU)
    )

    # Load the original image
    img = cv2.imread(input_image)

    if img is None:
        raise ValueError(f"Failed to load image: {input_image}")

    # Process results
    result = results[0]  # Single image, so take the first result
    boxes = result.boxes  # Bounding box objects

    # Draw bounding boxes, labels, and confidence scores
    for box in boxes:
        # Extract details
        cls_id = int(box.cls)  # Class ID
        label = model.names[cls_id]  # Class name ('L_tag' or 'square_tag')
        conf = box.conf.item()  # Confidence score
        x_min, y_min, x_max, y_max = map(int, box.xyxy[0])  # Bounding box coordinates

        # Draw rectangle (green, thickness=2)
        cv2.rectangle(img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

        # Add label and confidence above the box
        text = f"{label} {conf:.2f}"
        cv2.putText(
            img, text, (x_min, y_min - 10),  # Position above the box
            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2  # Font, size, color, thickness
        )

        # Print detection details
        print(f"Detected: {label}, Confidence: {conf:.3f}, Box: [{x_min}, {y_min}, {x_max}, {y_max}]")

    # Save the output image
    cv2.imwrite(output_image, img)
    print(f"Output saved to: {output_image}")

    # Optionally display the image
    if show_image:
        cv2.imshow("Prediction", img)
        cv2.waitKey(0)  # Wait for key press to close
        cv2.destroyAllWindows()


if __name__ == '__main__':
    # Ensure multiprocessing safety on Windows
    from multiprocessing import freeze_support

    freeze_support()

    # Run the prediction
    try:
        predict_and_draw(
            model_path=MODEL_PATH,
            input_image=INPUT_IMAGE,
            output_image=OUTPUT_IMAGE,
            conf_threshold=0.5,  # Adjust as needed
            imgsz=640,  # Matches your training size
            show_image=True  # Set to False to skip display
        )
    except Exception as e:
        print(f"Error: {e}")