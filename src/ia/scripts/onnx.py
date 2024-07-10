import os

import cv2
import torch
import torchvision.transforms as T
from PIL import Image


# Load the PyTorch model
def load_model(path):
	model = torch.load(
		path, map_location=torch.device("cuda" if torch.cuda.is_available() else "cpu")
	)
	model.eval()
	return model


# Load an array with all items from the asset folder
def load_assets(path):
	assets_folder = os.listdir(path)
	paths = []

	for asset in assets_folder:
		file_path = os.path.join(path, asset)
		paths.append(file_path)

	return paths


# Preprocess the image for the YOLO model
def preprocess_image(image_path, img_size=640):
	image = Image.open(image_path).convert("RGB")
	transform = T.Compose(
		[
			T.Resize((img_size, img_size)),
			T.ToTensor(),
			T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
		]
	)
	return transform(image).unsqueeze(0)


# Decode the output of the YOLO model
def decode_predictions(predictions, conf_thresh=0.5, iou_thresh=0.4):
	# Your YOLO model's postprocessing here, e.g. Non-Maximum Suppression (NMS)
	pass


# Create response images from the model's predictions
def detect(model, image_paths, img_size=640):
	predictions = []
	for image_path in image_paths:
		input_tensor = preprocess_image(image_path, img_size)
		with torch.no_grad():
			output = model(input_tensor)[0]
		output = decode_predictions(output)
		predictions.append(output)

	for i, r in enumerate(predictions):
		# Visualization can be implemented using OpenCV or PIL
		image = cv2.imread(image_paths[i])
		for box in r:
			x1, y1, x2, y2, conf, cls_id = box
			if conf > 0.5:
				cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
				cv2.putText(
					image,
					str(cls_id),
					(x1, y1),
					cv2.FONT_HERSHEY_SIMPLEX,
					1,
					(255, 0, 0),
					2,
				)

		save_path = f"./predictions_dirt/results{i}.jpg"
		cv2.imwrite(save_path, image)


def main():
	# Load the model
	model = load_model("./models/yolo_v8_n_dirt_detection.pt")

	# Load the images
	load_imgs = load_assets("./dirt")

	# Save the detections
	detect(model, load_imgs)


if __name__ == "__main__":
	main()
