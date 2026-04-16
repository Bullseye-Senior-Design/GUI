from typing import List
from pydantic import BaseModel

class BoundaryCorner(BaseModel):
    x: float
    y: float

class BoundaryData(BaseModel):
    corners: List[BoundaryCorner]
