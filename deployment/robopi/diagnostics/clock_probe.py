"""Read-only clock offset/step evidence with HDC round-trip uncertainty."""
import argparse
import json
from pathlib import Path
import shlex
import sys
import time

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'deployment/airsim/demos/super_forest'))
from robopi_execution import connect


def measure(link):
    start_mono=time.monotonic_ns()
    start_wall=time.time_ns()
    remote=json.loads(link.board('python3 -c '+shlex.quote(
        'import time,json;print(json.dumps({"wall_ns":time.time_ns(),"mono_ns":time.monotonic_ns()}))')))
    end_wall=time.time_ns()
    end_mono=time.monotonic_ns()
    return {'pc_wall_mid_ns':(start_wall+end_wall)//2,'pc_mono_mid_ns':(start_mono+end_mono)//2,
            'board_wall_ns':remote['wall_ns'],'board_mono_ns':remote['mono_ns'],
            'offset_seconds':(remote['wall_ns']-(start_wall+end_wall)//2)/1e9,
            'offset_lower_seconds':(remote['wall_ns']-end_wall)/1e9,
            'offset_upper_seconds':(remote['wall_ns']-start_wall)/1e9,
            'rtt_seconds':(end_mono-start_mono)/1e9,
            'pc_wall_step_during_request_seconds':((end_wall-start_wall)-(end_mono-start_mono))/1e9}


def assess(rows, offset_limit=.5, step_limit=.05):
    steps=[]
    for a,b in zip(rows,rows[1:]):
        board_step=((b['board_wall_ns']-a['board_wall_ns'])-(b['board_mono_ns']-a['board_mono_ns']))/1e9
        pc_step=((b['pc_wall_mid_ns']-a['pc_wall_mid_ns'])-(b['pc_mono_mid_ns']-a['pc_mono_mid_ns']))/1e9
        steps.append({'board_wall_minus_monotonic_change_seconds':board_step,
                      'pc_wall_minus_monotonic_change_seconds':pc_step})
    elapsed=(rows[-1]['pc_mono_mid_ns']-rows[0]['pc_mono_mid_ns'])/1e9 if len(rows)>1 else 0.
    delta=rows[-1]['offset_seconds']-rows[0]['offset_seconds']
    uncertainty=(rows[0]['rtt_seconds']+rows[-1]['rtt_seconds'])/2
    return {'samples':len(rows),'elapsed_seconds':elapsed,
            'definite_offset_limit_violation':any(r['offset_lower_seconds']>offset_limit or r['offset_upper_seconds'] < -offset_limit for r in rows),
            'wall_step_detected':any(abs(s['board_wall_minus_monotonic_change_seconds'])>step_limit or abs(s['pc_wall_minus_monotonic_change_seconds'])>step_limit for s in steps),
            'offset_change_seconds':delta,'offset_change_uncertainty_seconds':uncertainty,
            'observed_rate_ppm':delta/elapsed*1e6 if elapsed else None,
            'rate_uncertainty_ppm':uncertainty/elapsed*1e6 if elapsed else None,
            'rate_interval_ppm':[(delta-uncertainty)/elapsed*1e6,(delta+uncertainty)/elapsed*1e6] if elapsed else None,
            'frequency_drift_conclusion':'unresolved' if abs(delta)<=uncertainty else 'net offset change exceeds endpoint RTT bounds; investigate clock steps and frequency separately',
            'steps':steps,'limits':{'offset_seconds':offset_limit,'wall_step_seconds':step_limit},
            'scope':'Short read-only observation; gradual frequency drift needs longer observation. Linux monotonic can share clock slewing; wall/monotonic differences detect steps, not all drift.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples',type=int,default=12)
    parser.add_argument('--interval',type=float,default=5.)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.samples<2 or not 0<=args.interval<=30:parser.error('Require >=2 samples and interval in [0,30]')
    args.output.mkdir(parents=True,exist_ok=True)
    if (args.output/'samples.jsonl').exists():parser.error('Choose a fresh output directory')
    link=connect();rows=[]
    with (args.output/'samples.jsonl').open('w') as stream:
        for i in range(args.samples):
            row=measure(link);rows.append(row);stream.write(json.dumps(row)+'\n');stream.flush()
            if i+1<args.samples:time.sleep(args.interval)
    result=assess(rows)
    (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='steps'},indent=2))
    return int(result['definite_offset_limit_violation'] or result['wall_step_detected'])


if __name__=='__main__':sys.exit(main())
