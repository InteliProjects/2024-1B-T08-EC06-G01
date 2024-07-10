from database.postgres import base_ormar_config
from ormar import Boolean, Integer, Model, String


class User(Model):
	ormar_config = base_ormar_config.copy(tablename="users")

	id = Integer(primary_key=True, autoincrement=True)
	username = String(max_length=100)
	password = String(max_length=100)
	admin = Boolean()