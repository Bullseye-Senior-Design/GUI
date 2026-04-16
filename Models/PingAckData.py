from pydantic import BaseModel

class PingAckData(BaseModel):
    state_of_charge: float
