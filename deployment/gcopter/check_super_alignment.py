#!/usr/bin/env python3
"""Fail when GCOPTER's shared environment/acceptance semantics drift from SUPER."""
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
super_cfg = yaml.safe_load((ROOT/'deployment/airsim/demos/super_forest/planner.yaml').read_text())
gcopter_cfg = yaml.safe_load((ROOT/'src/tool/navigation/planner/gcopter/ros2_adapter/native/config/local_map.yaml').read_text())
super_task = json.loads((ROOT/'deployment/airsim/demos/super_forest/task.json').read_text())
local_source = (ROOT/'src/tool/navigation/planner/gcopter/ros2_adapter/native/src/local_node.cpp').read_text()
mission_source = (ROOT/'deployment/gcopter/mission.py').read_text()


def at(value, path):
    for key in path.split('.'):
        value = value[key]
    return value


# These belong to the shared sensor/environment model, not either planning core.
shared_map = (
    'rog_map.resolution', 'rog_map.inflation_resolution', 'rog_map.inflation_step',
    'rog_map.unk_inflation_en', 'rog_map.unk_inflation_step', 'rog_map.map_size',
    'rog_map.map_sliding.enable', 'rog_map.map_sliding.threshold',
    'rog_map.intensity_thresh', 'rog_map.point_filt_num',
    'rog_map.raycasting.enable', 'rog_map.raycasting.batch_update_size',
    'rog_map.raycasting.local_update_box', 'rog_map.raycasting.ray_range',
    'rog_map.raycasting.p_min', 'rog_map.raycasting.p_miss',
    'rog_map.raycasting.p_free', 'rog_map.raycasting.p_occ',
    'rog_map.raycasting.p_hit', 'rog_map.raycasting.p_max',
    'rog_map.raycasting.unk_thresh',
)
errors = []
for path in shared_map:
    expected, actual = at(super_cfg, path), at(gcopter_cfg, path)
    if actual != expected:
        errors.append(f'{path}: GCOPTER={actual!r}, SUPER={expected!r}')

# Keep success/collision/heading semantics comparable while allowing different planners.
goal_tolerance = float(super_task['goal_tolerance_m'])
source_checks = {
    f'declare_parameter("goal_tolerance",{goal_tolerance})': local_source,
    'std::atan2(velocity.y(),velocity.x())': local_source,
    f'(1.5 if self.local else .25)': mission_source,
    "self.speed() < .3": mission_source,
    "self.result['collision'] = collision": mission_source,
    "self.state.get('complete')": mission_source,
}
for needle, source in source_checks.items():
    if needle not in source:
        errors.append('missing acceptance/trajectory semantic: '+needle)

report = {
    'status': 'failed' if errors else 'passed',
    'reference': 'deployment/airsim/demos/super_forest',
    'shared_map_fields': len(shared_map),
    'goal_tolerance_m': goal_tolerance,
    'arrival_speed_m_s': 0.3,
    'aligned': [
        'ROG-Map occupancy and inflation parameters',
        'live LiDAR and sliding-map input model',
        'velocity-heading yaw',
        'goal tolerance and low-speed arrival',
        'AirSim collision event retention',
        'planner completion plus Behavior execution evidence',
    ],
    'intentional_differences': [
        'GCOPTER local route, FIRI corridor and MINCO/GCOPTER optimizer',
        'SUPER exploration and backup trajectory are not implemented',
        'GCOPTER target speed is about 3 m/s; SUPER forest target is 8 m/s',
        'GCOPTER currently uses its own rolling handoff schedule and stop policy',
    ],
    'errors': errors,
}
print(json.dumps(report, ensure_ascii=False, indent=2))
if errors:
    raise SystemExit(1)
