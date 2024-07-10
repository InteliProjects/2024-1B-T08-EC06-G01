import onnxruntime as ort
import numpy as np
import cv2


## Exemplo no github https://github.com/AndreyGermanov/yolov8_onnx_python/blob/main/object_detector.py

# Load ONNX model
model = ort.InferenceSession('../models/yolo_coco.onnx')

image_path = '../assets/dog-cat.jpg'

# Function to preprocess an image
def preprocess_image(image_path):
    # Read the image
    img = cv2.imread(image_path)

    # Convert BGR to RGB
    # img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Resize the image
    img_resized = cv2.resize(img, (640, 640))

    # Normalize the image
    img_normalized = img_resized.astype('float32') / 255.0

    # Add batch dimension and transpose the tensor
    img_batch = np.expand_dims(img_normalized, axis=0)
    img_transposed = np.transpose(img_batch, (0, 3, 1, 2))

    return img_transposed

# Preprocess the input image
processed_img = preprocess_image(image_path)

# Run inference
run_model = model.run(processed_img)
print(run_model)