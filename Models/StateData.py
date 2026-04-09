from typing import Optional
from enum import Enum 
from pydantic import BaseModel

class State(Enum):
    DISABLED = 0
    AUTONOMOUS = 1
    TELEOP = 2
    TEST = 3

class StateData(BaseModel):
    state: State
    path_speed: Optional[float]
    path_id: Optional[int]