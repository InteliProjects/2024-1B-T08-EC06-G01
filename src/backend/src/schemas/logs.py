from typing import Optional
from datetime import datetime
from pydantic import BaseModel, Field

class Log(BaseModel):
    id: int = Field(default=None, gt=0)
    date: Optional[datetime] = Field(default=None)
    emergency_button: bool = Field(default=False)
    ia_request: bool = Field(default=False)
    username: str = Field(default=None)
    user_id: int = Field(default=None)
