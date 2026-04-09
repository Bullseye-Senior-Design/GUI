from pydantic import BaseModel

class BatteryData(BaseModel):
    voltage: float
    current: float
    power: float
    state_of_charge: float
    time_remaining: float

# test 