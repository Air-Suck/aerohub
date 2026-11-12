"""Plot recorded navigation, never supplying inputs or commands to the flight stack."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser=argparse.ArgumentParser()
parser.add_argument('output',type=Path)
args=parser.parse_args()
result=json.loads((args.output/'result.json').read_text())
states=[]
for line in (args.output/'samples.jsonl').read_text().splitlines():
    sample=json.loads(line)
    if sample['topic']=='navigation_state':states.append(sample)
if not states:raise RuntimeError('no recorded navigation states')
origin=result['navigation_origin'];goal=result['target']
t=[s['sim']-states[0]['sim'] for s in states]
fig,axes=plt.subplots(1,3,figsize=(13,4))
axes[0].plot([s['p'][0]-origin[0] for s in states],[s['p'][1]-origin[1] for s in states],label='Actual estimate')
axes[0].plot([0,goal[0]-origin[0]],[0,goal[1]-origin[1]],'--',label='Start-goal line')
axes[0].scatter([goal[0]-origin[0]],[goal[1]-origin[1]],marker='x',label='Goal')
axes[0].set(xlabel='East displacement (m)',ylabel='North displacement (m)');axes[0].axis('equal');axes[0].legend()
axes[1].plot(t,[s['p'][2]-origin[2] for s in states],label='Height change')
axes[1].axhline(goal[2]-origin[2],linestyle='--',label='Goal');axes[1].set(xlabel='Simulation time (s)',ylabel='Height change (m)');axes[1].legend()
axes[2].plot(t,[s['goal_error_m'] for s in states],label='Estimated goal error')
axes[2].axhline(.3,linestyle='--',label='Arrival tolerance');axes[2].set(xlabel='Simulation time (s)',ylabel='Goal distance (m)');axes[2].legend()
for ax in axes:ax.grid(alpha=.25)
fig.suptitle('Real IPC forest: '+('PASS' if result['success'] else 'FAIL — '+str(result.get('error'))))
fig.tight_layout();fig.savefig(args.output/'navigation.png',dpi=160)
