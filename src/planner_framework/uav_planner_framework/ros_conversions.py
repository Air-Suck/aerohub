"""Conversions between ROS messages and the neutral trajectory model."""

from __future__ import annotations
from .registry import ConfigError

from .trajectory import (
    Setpoint,
    TimedTrajectorySegment,
    TrajectoryPoint,
    Vector3,
    VehicleTrajectory,
)


VELOCITY = 1
ACCELERATION = 2
YAW = 4


def time_to_seconds(value) -> float:
    return float(value.sec) + float(value.nanosec) / 1_000_000_000.0


def seconds_to_time(value: float, target) -> None:
    target.sec = int(value)
    target.nanosec = int(round((value - target.sec) * 1_000_000_000.0))
    if target.nanosec == 1_000_000_000:
        target.sec += 1
        target.nanosec = 0


def duration_to_seconds(value) -> float:
    return time_to_seconds(value)


def segment_from_msg(message) -> TimedTrajectorySegment:
    trajectories = []
    for item in message.vehicle_trajectories:
        if item.valid_fields & ~7:
            raise ConfigError("unknown trajectory valid_fields bits")
        fields = set()
        if item.valid_fields & VELOCITY:
            fields.add("velocity")
        if item.valid_fields & ACCELERATION:
            fields.add("acceleration")
        if item.valid_fields & YAW:
            fields.add("yaw")
        points = tuple(
            TrajectoryPoint(
                duration_to_seconds(point.time_from_start),
                Vector3(point.position.x, point.position.y, point.position.z),
                Vector3(point.velocity.x, point.velocity.y, point.velocity.z),
                Vector3(point.acceleration.x, point.acceleration.y, point.acceleration.z),
                point.yaw,
            )
            for point in item.points
        )
        trajectories.append(VehicleTrajectory(item.vehicle_id, frozenset(fields), points))
    return TimedTrajectorySegment(
        message.interface_version,
        message.task_id,
        time_to_seconds(message.generated_at),
        time_to_seconds(message.valid_from),
        message.is_final,
        tuple(trajectories),
    )


def fill_setpoint_message(message, vehicle_id: str, setpoint: Setpoint) -> None:
    message.vehicle_id = vehicle_id
    for target, source in (
        (message.position, setpoint.position),
        (message.velocity, setpoint.velocity),
        (message.acceleration, setpoint.acceleration),
    ):
        target.x, target.y, target.z = source.x, source.y, source.z
    message.yaw = setpoint.yaw
