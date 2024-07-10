from datetime import datetime
from typing import Optional
from uuid import UUID

from pydantic import BaseModel, Field


class Media(BaseModel):
    uuid: Optional[UUID] = Field(default=None)
    title: str = Field(default=None, max_length=100)
    type: bool = Field(default=False)
    date: Optional[datetime] = Field(default=None)
    robot_id: Optional[int] = Field(default=None)

