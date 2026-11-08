"""Neutral fleet trajectory contract and stateful gateway validation."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .registry import ConfigError


OPTIONAL_FIELDS = frozenset({"velocity", "acceleration", "yaw"})


@dataclass(frozen=True)
class Vector3:
    x: float
    y: float
    z: float

    def finite(self) -> bool:
        return all(math.isfinite(value) for value in (self.x, self.y, self.z))


@dataclass(frozen=True)
class TrajectoryPoint:
    time_from_start: float
    position: Vector3
    velocity: Vector3 | None = None
    acceleration: Vector3 | None = None
    yaw: float | None = None


@dataclass(frozen=True)
class VehicleTrajectory:
    vehicle_id: str
    valid_fields: frozenset[str]
    points: tuple[TrajectoryPoint, ...]


@dataclass(frozen=True)
class TimedTrajectorySegment:
    interface_version: int
    task_id: str
    generated_at: float
    valid_from: float
    is_final: bool
    vehicle_trajectories: tuple[VehicleTrajectory, ...]


@dataclass(frozen=True)
class Setpoint:
    position: Vector3
    velocity: Vector3
    acceleration: Vector3
    yaw: float


def _lerp(a: float, b: float, fraction: float) -> float:
    return a + (b - a) * fraction


def _lerp_vector(a: Vector3, b: Vector3, fraction: float) -> Vector3:
    return Vector3(_lerp(a.x, b.x, fraction), _lerp(a.y, b.y, fraction), _lerp(a.z, b.z, fraction))


def _lerp_yaw(a: float, b: float, fraction: float) -> float:
    delta = (b - a + math.pi) % (2.0 * math.pi) - math.pi
    return (a + delta * fraction + math.pi) % (2.0 * math.pi) - math.pi


class OutputGateway:
    def __init__(self, task_id: str, origins: dict[str, Vector3]):
        self.task_id = task_id
        self.origins = origins
        self.segment: TimedTrajectorySegment | None = None
        self._last_generated_at: float | None = None

    def accept(self, segment: TimedTrajectorySegment, now: float) -> bool:
        self._validate(segment, now)
        if self._last_generated_at is not None:
            if segment.generated_at == self._last_generated_at:
                return False
            if segment.generated_at < self._last_generated_at:
                raise ConfigError("out-of-order trajectory segment")
        self.segment = segment
        self._last_generated_at = segment.generated_at
        return True

    def clear(self) -> None:
        self.segment = None

    @property
    def coverage_end(self) -> float | None:
        if self.segment is None:
            return None
        return self.segment.valid_from + self.segment.vehicle_trajectories[0].points[-1].time_from_start

    def trajectory_timed_out(
        self, now: float, timeout: float, waiting_since: float | None = None
    ) -> bool:
        if self.segment is None:
            return waiting_since is not None and now - waiting_since > timeout
        if self.segment.is_final:
            return False
        end = self.coverage_end
        return end is not None and now - end > timeout

    def _validate(self, segment: TimedTrajectorySegment, now: float) -> None:
        if segment.interface_version != 1 or segment.task_id != self.task_id:
            raise ConfigError("trajectory interface or task_id mismatch")
        if not all(math.isfinite(value) for value in (segment.generated_at, segment.valid_from, now)):
            raise ConfigError("trajectory timestamps must be finite")
        trajectories = segment.vehicle_trajectories
        ids = [trajectory.vehicle_id for trajectory in trajectories]
        if len(ids) != len(set(ids)) or set(ids) != set(self.origins):
            raise ConfigError("trajectory vehicle set does not match task")
        end_times = []
        for trajectory in trajectories:
            if not trajectory.points:
                raise ConfigError(f"{trajectory.vehicle_id} trajectory is empty")
            if not trajectory.valid_fields <= OPTIONAL_FIELDS:
                raise ConfigError(f"{trajectory.vehicle_id} has unknown valid_fields")
            previous = -1.0
            for point in trajectory.points:
                if not math.isfinite(point.time_from_start) or point.time_from_start < 0 or point.time_from_start <= previous:
                    raise ConfigError(f"{trajectory.vehicle_id} times must be non-negative and strictly increasing")
                previous = point.time_from_start
                if not point.position.finite():
                    raise ConfigError(f"{trajectory.vehicle_id} position must be finite")
                for field in trajectory.valid_fields:
                    value = getattr(point, field)
                    finite = value.finite() if isinstance(value, Vector3) else value is not None and math.isfinite(value)
                    if not finite:
                        raise ConfigError(f"{trajectory.vehicle_id}.{field} is declared but invalid")
            end_times.append(trajectory.points[-1].time_from_start)
        if max(end_times) - min(end_times) > 1e-6:
            raise ConfigError("vehicle trajectories must have the same end time")
        if segment.valid_from + end_times[0] <= now:
            raise ConfigError("trajectory segment has no executable coverage")

    def sample(self, vehicle_id: str, now: float) -> Setpoint:
        if self.segment is None or vehicle_id not in self.origins:
            raise ConfigError("no active trajectory for vehicle")
        trajectory = next(item for item in self.segment.vehicle_trajectories if item.vehicle_id == vehicle_id)
        relative = max(0.0, now - self.segment.valid_from)
        before = after = trajectory.points[0]
        for point in trajectory.points:
            if point.time_from_start >= relative:
                after = point
                break
            before = point
        else:
            after = trajectory.points[-1]
        span = after.time_from_start - before.time_from_start
        fraction = 0.0 if span <= 0 else (relative - before.time_from_start) / span
        position = _lerp_vector(before.position, after.position, fraction)
        origin = self.origins[vehicle_id]
        position = Vector3(position.x - origin.x, position.y - origin.y, position.z - origin.z)
        nan_vector = Vector3(math.nan, math.nan, math.nan)
        velocity = (_lerp_vector(before.velocity, after.velocity, fraction)
                    if "velocity" in trajectory.valid_fields else nan_vector)
        acceleration = (_lerp_vector(before.acceleration, after.acceleration, fraction)
                        if "acceleration" in trajectory.valid_fields else nan_vector)
        yaw = (_lerp_yaw(before.yaw, after.yaw, fraction)
               if "yaw" in trajectory.valid_fields else math.nan)
        return Setpoint(position, velocity, acceleration, yaw)
