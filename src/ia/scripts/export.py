from ultralytics import YOLO


def main():
	# Carregando o modelo
	model = YOLO("./models/yolo_v8_n_dirt_detection.pt")

	# Exportando o modelo
	model.export(format="onnx")


if __name__ == "__main__":
	main()
