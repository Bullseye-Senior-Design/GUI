from pydantic import BaseModel

class DataPacket(BaseModel):
    type: str
    json_data: str