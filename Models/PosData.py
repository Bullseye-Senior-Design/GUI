from pydantic import BaseModel

class PosData(BaseModel):
    x: float
    y: float
    yaw: float
