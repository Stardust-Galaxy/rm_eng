import os
import cv2

def preprocess_images(input_dir, output_dir, image_size=(640, 480)):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    for image_file in os.listdir(input_dir):
        if image_file.endswith('.jpg') or image_file.endswith('.png'):
            image_path = os.path.join(input_dir, image_file)
            image = cv2.imread(image_path)

            # Convert to grayscale
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Resize image
            resized_image = cv2.resize(gray_image, image_size)

            # Normalize pixel values
            normalized_image = resized_image / 255.0

            # Save pre-processed image
            output_path = os.path.join(output_dir, image_file)
            cv2.imwrite(output_path, (normalized_image * 255).astype('uint8'))
def preprocess_image(image_path, image_size=(640, 480)):
    image = cv2.imread(image_path)

    # Convert to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Resize image
    resized_image = cv2.resize(gray_image, image_size)

    # Normalize pixel values
    normalized_image = resized_image / 255.0

    return normalized_image

def main():
    image_path = 'test.jpg'
    preprocessed_image = preprocess_image(image_path)
    output_path = 'preprocessed_test.jpg'
    cv2.imwrite(output_path, (preprocessed_image * 255).astype('uint8'))

if __name__ == "__main__":
    main()