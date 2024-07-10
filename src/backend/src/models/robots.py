from typing import Optional

from database.postgres import base_ormar_config
from models.users import User
from ormar import ForeignKey, Integer, Model, String


class Robot(Model):
	ormar_config = base_ormar_config.copy(tablename="robot")

	id = Integer(primary_key=True, autoincrement=True)
	name = String(max_length=100)
	user_id: Optional[User] = ForeignKey(User)