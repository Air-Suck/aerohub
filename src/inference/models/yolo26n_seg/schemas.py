from typing import Dict, List
from pydantic import BaseModel, ConfigDict, Field


class InferRequest(BaseModel):
    model_config = ConfigDict(extra='forbid')
    request_id: str = Field(min_length=1, max_length=128)
    stamp_ns: int = Field(gt=0)
    generation: int = Field(ge=0)
    frame: str = Field(min_length=1, max_length=256)
    width: int = Field(gt=0, le=2048)
    height: int = Field(gt=0, le=2048)
    encoding: str = 'png_bgr8'
    image_base64: str = Field(max_length=16777216)


class Detection(BaseModel):
    xyxy: List[float] = Field(min_length=4, max_length=4)
    confidence: float = Field(ge=0, le=1)
    category: str
    mask_png_base64: str = Field(default='',max_length=6291456)


class InferResponse(BaseModel):
    request_id: str
    stamp_ns: int
    generation: int
    frame: str
    width: int
    height: int
    model_version: str
    worker_generation: int
    boxes: List[Detection]
    transform: dict
    timings_ms: Dict[str, float]
