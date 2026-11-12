"""Verify actual ATTITUDE setpoints against paired AS2 actuator observations."""
import argparse
import collections
import json
import math
from pathlib import Path


def mul(a, b):
    w,x,y,z=a; v,i,j,k=b
    return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j, w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('output',type=Path)
    out=parser.parse_args().output
    data=collections.defaultdict(list)
    for line in (out/'samples.jsonl').read_text().splitlines():
        value=json.loads(line);data[value['topic']].append(value)
    source={r['stamp_ns']:r for r in data['external_acro/control']}
    thrust={r['stamp_ns']:r for r in data['actuator_command/thrust']}
    pairs=[(p,thrust[p['stamp_ns']]) for p in data['actuator_command/pose']
           if p['stamp_ns'] in thrust and p['stamp_ns'] in source]
    assert len(pairs)>=10,'missing same-source-stamp actuator pairs'
    max_force=max(abs(t['thrust']-source[p['stamp_ns']]['thrust']) for p,t in pairs)
    assert max_force<1e-5,'force differs from source'
    h=math.sqrt(.5)
    transformed=[(p['wall'],mul(mul([0,h,h,0],p['q']),[0,1,0,0]),t['thrust']) for p,t in pairs]
    errors=[]
    for value in data['fmu/in/vehicle_attitude_setpoint_v1']:
        candidates=[(q,force) for wall,q,force in transformed if abs(wall-value['wall'])<=.5]
        if not candidates: continue
        error=min(min(max(abs(a-b) for a,b in zip(q,value['q'])),max(abs(a+b) for a,b in zip(q,value['q'])))
                  +abs(value['thrust'][2]+force/15.) for q,force in candidates)
        errors.append(error)
    assert len(errors)>=10 and max(errors)<1e-5,'PX4 quaternion/normalized thrust mismatch'
    mode=(out/'px4-control-mode-uorb.txt').read_text()
    for flag in ['flag_control_offboard_enabled: True','flag_control_rates_enabled: True',
                 'flag_control_attitude_enabled: True','flag_control_velocity_enabled: False',
                 'flag_control_position_enabled: False']:
        assert flag in mode,flag
    result=json.loads((out/'result.json').read_text())
    assert result['success'] and result['landed_and_disarmed']
    report=dict(success=True,source_thrust_pairs=len(pairs),max_force_error_N=max_force,
                px4_samples=len(errors),max_quaternion_plus_throttle_error=max(errors),
                px4_attitude_loop_confirmed=True,
                limitation='PX4 matches use bounded 0.5s value window; no unique sequence identity. Validation source, not diffphys.')
    (out/'analysis.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))

if __name__=='__main__':main()
