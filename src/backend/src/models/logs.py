from datetime import datetime
from typing import Optional
from database.postgres import base_ormar_config
from ormar import Boolean, DateTime, ForeignKey, Integer, Model, String
from models.users import User

class Log(Model):
    ormar_config = base_ormar_config.copy(tablename="log")

    id = Integer(primary_key=True, autoincrement=True)
    date = DateTime(default=datetime.now)
    emergency_button = Boolean()
    ia_request = Boolean()
    username = String(max_length=100)
    user_id: Optional[User] = ForeignKey(User)


