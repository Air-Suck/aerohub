"""Actual-position completion checks shared by every planner."""

from __future__ import annotations

import math

from .trajectory import Vector3


class FleetCompletion:
    def __init__(
        self,
        targets: dict[str, Vector3],
        position_tolerance: float,
        hold_duration: float,
    ) -> None:
        self.targets = targets
        self.position_tolerance = position_tolerance
        self.hold_duration = hold_duration
        self.within_since: float | None = None

    def reset(self) -> None:
        self.within_since = None

    def update(self, positions: dict[str, Vector3], now: float) -> bool:
        if set(positions) != set(self.targets):
            self.within_since = None
            return False
        within = all(
            math.dist(
                (positions[vehicle].x, positions[vehicle].y, positions[vehicle].z),
                (target.x, target.y, target.z),
            )
            <= self.position_tolerance
            for vehicle, target in self.targets.items()
        )
        if not within:
            self.within_since = None
            return False
        if self.within_since is None:
            self.within_since = now
        return now - self.within_since >= self.hold_duration
