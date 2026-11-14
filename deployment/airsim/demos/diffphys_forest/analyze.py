"""Evaluate policy, sensor and task evidence without promoting failed stages."""
import argparse
import collections
import json
import statistics
from pathlib import Path


def analyze(out):
    result=json.loads((out/'result.json').read_text())
    observation=json.loads((out/'first-observation.json').read_text()) if (out/'first-observation.json').exists() else {}
    events=[json.loads(line) for line in (out/'policy.jsonl').read_text().splitlines()] if (out/'policy.jsonl').exists() else []
    depth=observation.get('depth',[])
    policies=[v for v in events if v['kind']=='policy']
    collision=result.get('collision_end',{})
    prior=result.get('collision_start',{})
    current_contract=result.get('stop_contract_version',1)>=2
    if current_contract:
        arrival_ok=(result.get('stop_wall_s',float('inf'))<=10 if result.get('expected_fault') else
            result.get('arrival_wall_s',float('inf'))<=result.get('arrival_wall_budget_s',0))
        stop_ok=(arrival_ok and result.get('terminal_settled',False) and
            result.get('low_speed_dwell_sim_s',0)>=2 and result.get('stop_speed_mps',float('inf'))<.3)
    else:
        # Historical runs retain their original combined ten-second gate.
        stop_ok=(result.get('stop_wall_s',float('inf'))<=10 and
            result.get('low_speed_dwell_sim_s',result.get('low_speed_dwell_wall_s',0))>=
                (2 if result.get('case')=='super-forest-route' else 1) and
            result.get('stop_speed_mps',float('inf'))<.3)
    criteria={
        'long_case_original_bounds':result.get('boundary_violations',0)==0,
        'real_GPU_policy':bool(policies) and all(v['device']=='cuda' for v in policies),
        'camera_dimensions':len(depth)==36 and all(len(r)==48 for r in depth),
        'no_close_body_returns_at_start':bool(depth) and all(v>=1.2 for r in depth for v in r),
        'horizon_obstacle_input':sum(1 for row in depth[10:26] for v in row if 1.2<v<15)>=10,
        'goal_1_5m_or_expected_fault':(result.get('goal_distance_m',float('inf'))<=1.5 if result.get('fault_scenario','none')=='none' else bool(result.get('expected_fault'))),
        'tilt_within_task_limit':result.get('tilt_max_rad',float('inf'))<=result.get('tilt_limit_rad',.6),
        'bounded_stop':stop_ok,
        'no_new_flight_collision':bool(collision) and (not collision.get('has_collided') or collision.get('time_stamp')==prior.get('time_stamp')),
        'landed_disarmed':bool(result.get('landed_and_disarmed')),
        'task_success':bool(result.get('success')),
        'owned_resources_cleaned':json.loads((out/'cleanup.json').read_text()).get('owned_resources_stopped',False) if (out/'cleanup.json').exists() else False,
    }
    if current_contract:
        criteria['execution_stop_acknowledged']=(result.get('execution_stop_confirmed',False) and
            result.get('execution_stop_wall_s',float('inf'))<=10)
    report=dict(success=all(criteria.values()),criteria=criteria,policy_calls=len(policies),
                observed_latency_sim_s=dict(median=statistics.median([v['latency'] for v in policies]) if policies else None,
                                            maximum=max([v['latency'] for v in policies],default=None)),
                input_wait_counts=dict(collections.Counter(v.get('error','') for v in events if v['kind']=='input_wait')),
                policy_profile=result.get('policy_profile','swarm'),
                stop_contract_version=result.get('stop_contract_version',1),
                scope='Diagnostic task gates only; not full reproduction acceptance. Camera/hover mapping and model preprocessing require separate equivalence validation.')
    (out/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
    return report

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('output',type=Path)
    r=analyze(parser.parse_args().output);print(json.dumps(r,indent=2));raise SystemExit(0 if r['success'] else 1)
