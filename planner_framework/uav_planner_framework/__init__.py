"""Shared planner configuration and trajectory contracts."""

from .behavior import BehaviorLifecycle, BehaviorSnapshot, BehaviorStatus, TransitionError
from .completion import FleetCompletion
from .registry import ConfigError, PlannerRegistry, recursive_merge
from .trajectory import (
    OutputGateway,
    TimedTrajectorySegment,
    TrajectoryPoint,
    Vector3,
    VehicleTrajectory,
)

__all__ = [
    "BehaviorLifecycle", "BehaviorSnapshot", "BehaviorStatus", "ConfigError", "FleetCompletion",
    "OutputGateway", "PlannerRegistry", "TimedTrajectorySegment", "TrajectoryPoint",
    "TransitionError", "Vector3", "VehicleTrajectory", "recursive_merge",
]
