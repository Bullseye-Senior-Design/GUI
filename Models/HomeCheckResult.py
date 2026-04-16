from pydantic import BaseModel

class HomeCheckResult(BaseModel):
    ok: bool
