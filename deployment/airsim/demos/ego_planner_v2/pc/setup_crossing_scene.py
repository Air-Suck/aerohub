import os
"""Ground-only test setup; caller must stop both PX4 instances and AS2 first.

Never imported by a flight mission. Save before/after state; not flight evidence.
"""
import json, os, sys, time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).with_name('vendor')))
os.environ['MSGPACK_PUREPYTHON']='1'
from airsim_rpc import Rpc

rpc=Rpc('host.docker.internal',41451,3.)
report={'scope':'ground scene preparation only; NOT flight acceptance','before':{},'after':{}}
for i in (0,1):
    name=f'Drone{i}'
    state=rpc.call('simGetGroundTruthKinematics',name)
    report['before'][name]=state
rpc.call('reset')
rpc.call('simPause',False)
for i in (0,1):report['after'][f'Drone{i}']=rpc.call('simGetGroundTruthKinematics',f'Drone{i}')
Path('/evidence/scene-setup.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
assert all(abs(v['position']['x_val'])<.15 and abs(v['position']['y_val'])<.15
    and -.2<v['position']['z_val']<.8 for v in report['after'].values()), 'scene setup did not settle at spawn'
