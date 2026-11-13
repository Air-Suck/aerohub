"""Record wall-clock uncertainty; optional explicitly authorized disarmed-only sync."""
import argparse
import json
from pathlib import Path
import time
import run

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('output',type=Path)
parser.add_argument('--sync-if-needed',action='store_true')
args=parser.parse_args()
config=json.loads((args.output/'config.json').read_text())

def probe():
    rows=[]
    for _ in range(5):
        start=time.time()
        board=float(run.board_shell(config,"python3 -c 'import time; print(time.time())'",timeout=5))
        end=time.time()
        rows.append(dict(pc_before=start,pc_after=end,board=board,rtt=end-start,offset=board-(start+end)/2))
    return rows

before=probe(); result=dict(before=before)
best=min(before,key=lambda r:r['rtt'])
if abs(best['offset'])+best['rtt']/2>.5 and args.sync_if_needed:
    state=run.board_action(config,'status')
    if not state['status_fresh'] or any(state['status'].get(k) is not False for k in ('armed','px4_armed')):
        raise RuntimeError('Fresh disarmed state required for authorized clock sync')
    result['disarmed_state']=state
    target=time.time()+best['rtt']/2
    result['sync']=run.board_shell(config,"python3 -c 'import time; time.clock_settime(time.CLOCK_REALTIME,"+str(target)+"); print(time.time())'",timeout=5)
    result['after']=probe()
best=min(result.get('after',before),key=lambda r:r['rtt'])
result['bound_s']=abs(best['offset'])+best['rtt']/2
result['passed']=result['bound_s']<=.5
run.write(args.output/('clock-preflight-'+str(time.time_ns())+'.json'),result)
print(json.dumps(result,indent=2))
raise SystemExit(0 if result['passed'] else 1)
