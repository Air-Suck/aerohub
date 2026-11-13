"""Explicit recurrent state: a timed-out request never mutates worker state."""

from typing import List, Optional, Dict
from pydantic import BaseModel, ConfigDict, Field, field_validator


class InferRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    request_id: str = Field(min_length=1, max_length=128)
    # AirSim release: 36 x 48 DepthPlanar meters, pooled by three.
    depth: List[List[float]] = Field(min_length=36, max_length=36)
    state: List[float] = Field(min_length=10, max_length=10)
    hidden_state: Optional[List[float]] = Field(default=None, min_length=192, max_length=192)

    @field_validator("depth")
    @classmethod
    def check_depth(cls, rows):
        if any(len(row) != 48 or any(value <= 0 for value in row) for row in rows):
            raise ValueError("depth must be 36x48 positive finite DepthPlanar meters")
        return rows


class InferResponse(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False)
    request_id: str
    model_version: str
    device: str
    action: List[float] = Field(min_length=6, max_length=6)
    hidden_state: List[float] = Field(min_length=192, max_length=192)
    timings_ms: Dict[str, float]
