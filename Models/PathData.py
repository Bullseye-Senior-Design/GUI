from pydantic import BaseModel

class PathData(BaseModel):
    position_list: list[tuple[float, float]]
    id: int