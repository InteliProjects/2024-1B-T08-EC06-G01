import asyncio
import json
import os

from utils.crypto import get_password_hash
from models.robots import Robot as RobotModel
from models.users import User as UserModel
from models.temp import Temp as TempModel
import websockets
from fastapi import WebSocket
from websockets.exceptions import ConnectionClosedError
from pytz import timezone
from datetime import datetime

ROBOT_WEBSOCKET_URL = os.environ.get('ROBOT_WEBSOCKET_URL') or ""

class Robot:
	def __init__(self):
		self.websocket = None
		self.clients = set()

	async def connect(self):
		"""Connect to the robot WebSocket and start listening for messages."""
		print("Connecting to robot websocket...")
		self.websocket = await websockets.connect(ROBOT_WEBSOCKET_URL)
		asyncio.create_task(self._broadcast_forever())

	async def _broadcast_forever(self):
		"""Listen for messages from the WebSocket and broadcast them to all connected clients."""
		try:
			if not await UserModel.objects.get(id=1):
				await UserModel.objects.create(
					id = 1,
					username="admin",
					password=get_password_hash('admin'),
					admin = True
				)
		except:
			await UserModel.objects.create(
				id = 1,
				username="admin",
				password=get_password_hash('admin'),
				admin = True
			)

		try:
			if not await RobotModel.objects.get(id=1):
				await RobotModel.objects.create(
					id = 1,
					name="Default Robot",
					user_id=1
				)
		except:
			await RobotModel.objects.create(
				id = 1,
				name="Default Robot",
				user_id=1
			)

		try:
			async for message in self.websocket: # type: ignore
				await self._broadcast(message)

				jsonified = json.loads(message)
				if 'temperature' in jsonified and 'position' in jsonified:

					current_time = datetime.now(tz=timezone('America/Sao_Paulo'))
					current_time_naive = current_time.replace(tzinfo=None)

					await TempModel.objects.create(
						temp=float(jsonified['temperature']),
						location_x = jsonified['position']['x'],
						location_y = jsonified['position']['y'],
						date=current_time_naive,
						robot_id=1 # todo change
					)

		except ConnectionClosedError:
			print("Connection to robot has been lost, attempting to reconnect...")
			await self._broadcast(json.dumps({ "type": "SPacketError", "data": {
				"message": "Conexão com o robô foi perdida"
			}}))
			await self.reconnect()

	async def add_client(self, client: WebSocket):
		self.clients.add(client)
		print(f"Added client {client}")

	async def remove_client(self, client: WebSocket):
		self.clients.remove(client)
		print(f"Removed client {client}")

	async def send(self, data):
		"""Send a message to the robot."""
		if self.websocket is None:
			raise Exception("Not connected to robot")

		await self.websocket.send(data)

	async def _broadcast(self, message):
		print(f"Broadcasting message to {len(self.clients)} clients: {message}")
		if self.clients:
			await asyncio.gather(*[client.send_text(message) for client in self.clients])

	async def _reconnect(self):
		if self.websocket:
			try: await self.close()
			except: pass

		try: await self.connect()
		except Exception as e:
			print(f"Failed to reconnect to robot: {e}, retrying in 5 seconds...")
			await asyncio.sleep(5)
			return await self.reconnect()

		await self._broadcast(json.dumps({ "type": "SPacketInfo", "data": {
			"message": "Conexão com o robô estabelecida"
		}}))
		print("Connected to robot")

	async def reconnect(self):
		"""Close the current connection and reconnect to the robot WebSocket."""
		# use _reconnect in a non-blocking way
		asyncio.create_task(self._reconnect())

	async def close(self):
		if self.websocket:
			await self.websocket.close()
		self.websocket = None


robot = Robot()