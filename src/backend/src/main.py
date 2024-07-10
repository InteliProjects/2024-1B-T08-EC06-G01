import os
from contextlib import asynccontextmanager

import uvicorn
from dotenv import load_dotenv
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from client.robot import robot

# from client.camera import camera
from database.postgres import database
from routes.router import router

load_dotenv("./.env")

assert os.environ.get("DATABASE_URL") != ""
assert os.environ.get("BUCKET_HOST") != ""
assert os.environ.get("BUCKET_PORT") != ""
assert os.environ.get("BUCKET_ACCESS_KEY") != ""
assert os.environ.get("BUCKET_SECRET_KEY") != ""
assert os.environ.get("BUCKET_USE_SSL") != ""
assert os.environ.get("JWT_SECRET") != ""
assert os.environ.get("AES_SECRET") != ""
assert os.environ.get("ROBOT_WEBSOCKET_URL") != ""
assert os.environ.get("CAMERA_WEBSOCKET_URL") != ""


@asynccontextmanager
async def lifespan(app: FastAPI):
	database_ = app.state.database
	robot_ = app.state.robot
	try:
		try:
			print("Connecting to database...")
			await database_.connect()
			print("Database connected")
		except Exception as e:
			print(f"Error connecting to database: {e}")

		try:
			await robot_.reconnect()
			print("Robot connected")
		except Exception as e:
			print(
				f"Error connecting to robot, please check if the robot is online: {e}"
			)

		# try:
		#     await camera.connect()
		#     print("Camera connected")
		# except Exception as e:
		#     print(f"Error connecting to camera, please check if the camera is online: {e}")

		yield
	finally:
		if robot_.websocket:
			await robot_.close()

		# if camera.websocket: await camera.close()

		if database_.is_connected:
			await database_.disconnect()


app = FastAPI(lifespan=lifespan)

app.state.database = database
app.state.robot = robot
# app.state.camera = camera


app.include_router(router)


# Permitindo requisições do front
app.add_middleware(
	CORSMiddleware,
	allow_origins=["*"],  # Allows all origins
	allow_credentials=True,
	allow_methods=["*"],  # Allows all methods
	allow_headers=["*"],  # Allows all headers
)


if __name__ == "__main__":
	uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
