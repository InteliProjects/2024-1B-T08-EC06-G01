from typing import Literal

import pydantic


class ControlData(pydantic.BaseModel):
	state: Literal["forward", "backward", "left", "right", "stopped", "emergency"]

class ControlPacket(pydantic.BaseModel):
	type: Literal["CPacketControl"]
	data: ControlData
