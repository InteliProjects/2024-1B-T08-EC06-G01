# Utilizando o modelo do como torchscript

import cv2
import torch
from PIL import Image
import torchvision.transforms as transforms
import os


# Carregando o modelo 
def load_model():
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    pathzin = '../models/model_v8.torchscript'
    model = torch.jit.load(pathzin)

    # model = torch.hub.load(model='../models/yolo_v8_n.pt', source='local') 
    return model

def count_detections(output, threshold=0.5):
    # Assuming the model's output format is [batch_size, num_detections, 85]
    # where 85 = [x, y, w, h, confidence, class scores...]
    
    # Extract the confidence scores
    confidences = output[..., 4]
    
    # Filter out detections with confidence below the threshold
    mask = confidences > threshold
    
    # Extract class scores and get the index of the maximum score for each detection
    class_scores = output[..., 5:]
    class_ids = torch.argmax(class_scores, dim=-1)
    
    # Apply the mask to filter out low-confidence detections
    class_ids = class_ids[mask]
    
    # Count occurrences of each class ID
    unique, counts = torch.unique(class_ids, return_counts=True)
    detection_counts = dict(zip(unique.cpu().numpy(), counts.cpu().numpy()))
    
    return detection_counts



def main():
    # Modelinho
    pathzin_img = '../assets/city.jpg'

    # Carregando o modelo
    model = load_model()

    transform = transforms.Compose([
    transforms.Resize((640, 640)),
    transforms.ToTensor(),
    ])


    # Load and preprocess input image
    input_image = Image.open(pathzin_img)
    input_tensor = transform(input_image)
    input_batch = input_tensor.unsqueeze(0)  # Add batch dimension

    # Run inference
    output = model(input_batch)
    print("output 1", output)

    # Reshape output tensor to match the expected shape
    # output = output.view(1, 84, 8400)

    # print("output 2",output)

    detection_counts = count_detections(output[0])  # Assuming batch size of 1

    print("Detected objects:", detection_counts)



if __name__ == "__main__":
    main()