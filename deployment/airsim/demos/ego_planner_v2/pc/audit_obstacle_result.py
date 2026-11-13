"""Independent offline audit of collision timestamps and real flight metrics."""
import json,math
from pathlib import Path
import numpy as np

root=Path('/evidence');report=json.loads((root/'result.json').read_text())
rows=[json.loads(line) for line in (root/'flight.jsonl').read_text().splitlines()]
ego=[r for r in rows if r['phase']=='ego']
before=[r for r in rows if r['phase'] in ('preflight','arming','taking_off','settling','holding') and 'collision' in r]
collision=[r for r in ego if 'collision' in r]
baseline=before[-1]['collision']['time_stamp']
events=[r for r in collision if r['collision']['time_stamp']!=baseline]
collision_coverage=len(collision)>=20
if report.get('collision_observation'):
    checkpoints={r['collision_checkpoint']:r for r in rows if 'collision_checkpoint' in r}
    baseline=checkpoints['before_ego']['collision']['time_stamp']
    collision=[checkpoints['before_ego'],checkpoints['after_ego']]
    events=[r for r in collision if r['collision']['time_stamp']!=baseline]
    collision_coverage=checkpoints['before_ego']['wall']<=ego[0]['wall'] and checkpoints['after_ego']['wall']>=ego[-1]['wall']
odom=[r for r in ego if r.get('topic')=='odom']
refs=json.loads((root/'references.json').read_text())
track=np.asarray(json.loads((root/'actual-track.json').read_text()))
truth=[r for r in ego if 'scene_pose' in r]
truth_xy=np.asarray([[r['scene_pose']['position']['y_val'],r['scene_pose']['position']['x_val']] for r in truth])
start=np.asarray(report['ego_start_position'][:2]);goal=np.asarray(report['goal_enu'][:2])
direction=(goal-start)/np.linalg.norm(goal-start);perpendicular=np.array([-direction[1],direction[0]])
truth_lateral=abs((truth_xy-start)@perpendicular)
position=np.asarray([r['position'] for r in refs]);actual=np.asarray([r['actual'] for r in refs])
map_obstacle_counts={}
for key in ('map','inflated'):
    mapped=np.load(root/(key+'-before-goal.npy'))
    center=np.asarray(report['obstacle_center_xy'])
    map_obstacle_counts[key]=int(np.count_nonzero((np.linalg.norm(mapped[:,:2]-center,axis=1)<1.)&(mapped[:,2]>.5)))
summary={'passed':False,'collision_observations':len(collision),'collision_timestamp_before_ego':baseline,
    'collision_timestamps_during_ego':sorted(set(r['collision']['time_stamp'] for r in collision)),
    'new_collision_timestamp_events':len(events),'ego_map_obstacle_points_before_goal':map_obstacle_counts,
    'trajectory_duration_sim_s':ego[-1]['sim']-ego[0]['sim'],
    'measured_path_length_m':float(np.linalg.norm(np.diff(track[:,1:],axis=0),axis=1).sum()),
    'scene_pose_max_lateral_detour_m':float(truth_lateral.max()),
    'actual_speed_max_m_s':max(r['speed'] for r in odom),
    'reference_velocity_max_m_s':max(math.sqrt(sum(v*v for v in r['velocity'])) for r in refs),
    'reference_acceleration_max_m_s2':max(math.sqrt(sum(a*a for a in r['acceleration'])) for r in refs),
    'sampled_tracking_error_max_m':float(np.linalg.norm(position-actual,axis=1).max()),
    'scope':'Scene pose is observer-only. Cloud clearance is distance to observed points, not a complete surface-distance guarantee.'}
summary['passed']=bool(report['passed'] and collision_coverage and not events and all(map_obstacle_counts.values()) and summary['reference_velocity_max_m_s']<=.2415 and summary['reference_acceleration_max_m_s2']<=.3675 and summary['scene_pose_max_lateral_detour_m']>=.6)
(root/'independent-audit.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
raise SystemExit(0 if summary['passed'] else 1)
