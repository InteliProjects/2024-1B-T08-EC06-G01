from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime

class Temp(BaseModel):
	id: int = Field(default=None, gt=0)
	temp: float = Field(default=None)
	date: Optional[datetime] = Field(default=None)
	location_x: float = Field(default=None)
	location_y: float = Field(default=None)
	robot_id: int = Field(default=None)