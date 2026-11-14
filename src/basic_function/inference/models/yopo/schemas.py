"""YOPO model contract; vectors use the aligned x-forward/y-left/z-up frame."""

import math
from typing import List, Dict

from pydantic import BaseModel, ConfigDict, Field, field_validator


class InferRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    request_id: str = Field(default="", max_length=128)
    depth: List[List[float]]
    state: List[float] = Field(min_length=9, max_length=9)

    @field_validator("depth")
    @classmethod
    def check_depth(cls, rows):
        if not rows or not rows[0] or len(rows) * len(rows[0]) > 1048576:
            raise ValueError("depth must contain 1..1048576 pixels")
        width = len(rows[0])
        if any(len(row) != width for row in rows):
            raise ValueError("depth must be rectangular")
        if any(not math.isfinite(value) for row in rows for value in row):
            raise ValueError("depth must contain finite meters (zero denotes invalid depth)")
        return rows


class InferResponse(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False)
    request_id: str
    model_version: str
    device: str
    best_index: int
    best_endstate: List[float]
    endstates: List[List[float]]
    scores: List[float]
    duration_s: float
    timings_ms: Dict[str, float]
