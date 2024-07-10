from pydantic import BaseModel, Field


class User(BaseModel):
	id: int = Field(default=None, gt=0)
	username: str = Field(default=None, max_length=100)
	password: str = Field(default=None, max_length=100)
	admin: bool = Field(default=False)


