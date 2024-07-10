from pydantic import BaseModel, Field


class Robot(BaseModel):
	id: int = Field(default=None, gt=0)
	name: str = Field(default=None, max_length=100)
	user_id: int = Field(default=None)