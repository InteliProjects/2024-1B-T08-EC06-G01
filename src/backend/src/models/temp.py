from typing import Optional
from datetime import datetime

from database.postgres import base_ormar_config
from models.robots import Robot
from ormar import ForeignKey, Integer, Model, Float, DateTime

class Temp(Model):
    ormar_config = base_ormar_config.copy(tablename="temperature")
    
    id = Integer(primary_key=True, autoincrement=True)
    temp = Float()
    location_x = Float()
    location_y = Float()
    robot_id: Optional[Robot] = ForeignKey(Robot)
    date = DateTime(default=datetime.now)
