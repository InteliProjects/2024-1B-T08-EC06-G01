import uuid
from datetime import datetime
from typing import Optional

from database.postgres import base_ormar_config
from models.robots import Robot
from ormar import UUID, Boolean, DateTime, ForeignKey, Model, String


class Media(Model):
	ormar_config = base_ormar_config.copy(tablename="media")

	uuid = UUID(primary_key=True, default=uuid.uuid4)
	title = String(max_length=100)
	type = Boolean()
	date = DateTime(default=datetime.now)
	robot_id: Optional[Robot] = ForeignKey(Robot)