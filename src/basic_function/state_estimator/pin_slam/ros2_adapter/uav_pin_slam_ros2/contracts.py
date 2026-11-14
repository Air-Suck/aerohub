from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class InputDecision:
    accepted: bool
    reason: str = ""


class PointCloudContract:
    """Stateful checks applied before a cloud reaches the GPU algorithm."""

    def __init__(self, expected_frame: str) -> None:
        if not expected_frame:
            raise ValueError("expected_frame must not be empty")
        self.expected_frame = expected_frame
        self._last_stamp_ns: Optional[int] = None

    def check(self, frame_id: str, stamp_ns: int, point_count: int) -> InputDecision:
        if frame_id != self.expected_frame:
            return InputDecision(False, "unexpected_frame")
        if stamp_ns <= 0:
            return InputDecision(False, "invalid_stamp")
        if self._last_stamp_ns is not None and stamp_ns <= self._last_stamp_ns:
            return InputDecision(False, "non_monotonic_stamp")
        if point_count < 10:
            return InputDecision(False, "insufficient_points")
        self._last_stamp_ns = stamp_ns
        return InputDecision(True)


def xyz_fields_valid(fields, float32_datatype: int = 7) -> bool:
    """Require one unambiguous float32 field for each AirSim XYZ coordinate."""
    coordinates = {"x": 0, "y": 0, "z": 0}
    for field in fields:
        if field.name in coordinates:
            if field.datatype != float32_datatype or field.count != 1:
                return False
            coordinates[field.name] += 1
    return all(count == 1 for count in coordinates.values())
