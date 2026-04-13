from pydantic import BaseModel

class KFXSpeedData(BaseModel):
    speed: float  # 0.0–1.0
