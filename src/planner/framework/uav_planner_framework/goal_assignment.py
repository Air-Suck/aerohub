"""Strict vehicle-to-slot matching; never renumber or reorder vehicle trajectories."""
import math
from .registry import ConfigError


def match_goal_slots(endpoints, slots, policy='fixed', tolerance=.001):
    if policy not in ('fixed','optimize'):
        raise ConfigError('unknown goal assignment policy')
    if set(endpoints)!=set(slots) or not slots:
        raise ConfigError('goal assignment vehicle/slot set mismatch')
    for point in [*endpoints.values(),*slots.values()]:
        if len(point)!=3 or not all(math.isfinite(x) for x in point):
            raise ConfigError('goal assignment requires finite three-dimensional endpoints')
    result={}
    for vehicle,point in endpoints.items():
        candidates=[slot for slot,target in slots.items() if math.dist(point,target)<=tolerance]
        if len(candidates)!=1:
            raise ConfigError('unmatched or ambiguous goal for '+vehicle)
        slot=candidates[0]
        if policy=='fixed' and slot!=vehicle:
            raise ConfigError('planner reassigned a fixed per-vehicle goal: '+vehicle)
        result[vehicle]=slot
    if len(set(result.values()))!=len(slots):
        raise ConfigError('goal assignment must use every requested slot exactly once')
    return result
