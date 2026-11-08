#!/usr/bin/env python3
"""Analyze recorded data; no ROS, control, or changes to original result.json."""
import argparse
import json
import math
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/planner/super/ros2_adapter'))
from uav_super_adapter.polynomial import Polynomial


def analyze(folder):
    rows=[json.loads(line) for line in (folder/'flight.jsonl').read_text().splitlines()]
    origin=json.loads((folder/'origin-measured.json').read_text())['task_to_odom_translation_m']
    task=json.loads((Path(__file__).parent/'source-task.json').read_text())
    route=[r for r in rows if r.get('phase')=='route']
    if not route:raise ValueError('no route observations')
    points=[[a-b for a,b in zip(r['position'],origin)] for r in route]
    outside=[any(v<a or v>b for v,a,b in zip(p,task['bounds_enu_min_m'],task['bounds_enu_max_m'])) for p in points]
    duration=sum(max(0,b['sim_time']-a['sim_time']) for a,b,violating in zip(route,route[1:],outside) if violating)
    plans=[]
    for row in rows:
        if row.get('kind')=='plan':
            plans.append((row['generated'],Polynomial(row['start'],row['durations'],row['axes'],row['order'])))
    references=[r for r in rows if r.get('kind')=='reference']
    matched=0;errors=[0.,0.,0.]
    for ref in references:
        candidates=[]
        for generated,curve in plans:
            if generated<=ref['stamp']+.05 and curve.start<=ref['stamp']<=curve.end:
                vectors=curve.sample(ref['stamp'])
                differences=[math.dist(ref[key],vector) for key,vector in zip(('p','v','a'),vectors)]
                candidates.append(differences)
        if candidates:
            best=min(candidates,key=lambda x:max(x[0]/.05,x[1]/.1,x[2]))
            errors=[max(a,b) for a,b in zip(errors,best)];matched+=1
    result=json.loads((folder/'result.json').read_text())
    return {'result_success':result.get('success',False),
            'axis_min_max_task_enu_m':[[min(p[i] for p in points),max(p[i] for p in points)] for i in range(3)],
            'observed_route_length_m':sum(math.dist(a,b) for a,b in zip(points,points[1:])),
            'bounds_policy':task['trajectory_limits_action'],
            'out_of_bounds_samples':sum(outside),'sampled_out_of_bounds_seconds_estimate':duration,
            'strict_bounds_pass_for_observed_samples':not any(outside),
            'source_flag_counts':result.get('source_flags',{}),
            'native_plans_recorded':len(plans),'references_recorded':len(references),
            'references_with_source_curve_candidate':matched,
            'max_best_match_pva_difference':errors,
            'matching_note':'Best match across received native plans at reference header time; includes interpolation and publication timing error, and does not prove a unique revision mapping.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path)
    args=parser.parse_args();result=analyze(args.folder)
    (args.folder/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
