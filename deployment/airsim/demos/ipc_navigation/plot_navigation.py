"""Plot recorded navigation and scoring truth; never publishes flight commands."""
import argparse
import json
import math
from pathlib import Path
import sqlite3

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Path as NavigationPath

p=argparse.ArgumentParser(description=__doc__);p.add_argument('evidence',type=Path);a=p.parse_args();b=a.evidence
result=json.loads((b/'result.json').read_text());task=json.loads((b/'forest-task.json').read_text())
nav=[];truth=[]
for line in (b/'samples.jsonl').read_text().splitlines():
 record=json.loads(line)
 if record['topic']=='navigation_state':nav.append(record)
 if record['topic']=='airsim_truth':truth.append(record)
if not nav or not truth:raise RuntimeError('recorded navigation and truth samples required')
ground=result['task_ground_origin_enu'];gt=result['task_ground_origin_truth_ned'];start=result['navigation_origin'];goal=result['target']
relative=lambda point:[v-g for v,g in zip(point,ground)]
estimated=[relative(r['p']) for r in nav]
truth_local=[[r['p_ned'][1]-gt['y_val'],r['p_ned'][0]-gt['x_val'],gt['z_val']-r['p_ned'][2]] for r in truth]
plans=[];height_deviation=0.
delta=[goal[i]-start[i] for i in range(3)];horizontal=delta[0]**2+delta[1]**2
for db in sorted((b/'rosbag').glob('*.db3')):
 with sqlite3.connect('file:'+str(db.resolve())+'?mode=ro',uri=True) as c:
  for payload, in c.execute("select m.data from messages m join topics t on t.id=m.topic_id where t.name='/drone0/ipc/path' order by m.timestamp"):
   message=deserialize_message(payload,NavigationPath)
   if message.header.frame_id!='earth':raise ValueError('unknown path frame')
   points=[]
   for pose in message.poses:
    q=pose.pose.position;point=[q.x,q.y,q.z];points.append(relative(point))
    alpha=max(0.,min(1.,sum((point[i]-start[i])*delta[i] for i in (0,1))/horizontal))
    height_deviation=max(height_deviation,abs(point[2]-start[2]-alpha*delta[2]))
   if points:plans.append(points)
fig=plt.figure(figsize=(13,9));xy=fig.add_subplot(221);xyz=fig.add_subplot(222,projection='3d');speed=fig.add_subplot(223);height=fig.add_subplot(224)
for points in plans[::max(1,len(plans)//30)]:
 xy.plot([q[0]for q in points],[q[1]for q in points],color='gray',alpha=.2,linewidth=.6)
xy.plot([q[0]for q in estimated],[q[1]for q in estimated],label='Estimated state')
xy.plot([q[0]for q in truth_local],[q[1]for q in truth_local],label='Scoring truth',linestyle='--')
target=relative(goal);xy.scatter([0,target[0]],[0,target[1]],marker='x',color='red');xy.set(xlabel='East / m',ylabel='North / m',title='SUPER A to B, free XYZ path');xy.axis('equal');xy.legend()
xyz.plot(*zip(*estimated),label='Estimated');xyz.plot(*zip(*truth_local),linestyle='--',label='Scoring truth');xyz.set(xlabel='East / m',ylabel='North / m',zlabel='Ground-relative Up / m',title='3D recorded motion')
times=[r['sim']-nav[0]['sim'] for r in nav]
speed.plot(times,[r['speed_mps']for r in nav],label='Measured state speed');speed.axhline(task['nominal_cruise_m_s'],color='red',linestyle='--',label='Nominal cruise');speed.axhline(task['cruise_min_m_s'],color='gray',linestyle=':',label='Cruise threshold');speed.set(xlabel='Simulation time since first navigation sample / s',ylabel='Speed / m/s');speed.legend()
height.plot(times,[q[2]for q in estimated],label='Estimated Up');height.plot([r['sim']-nav[0]['sim']for r in truth],[q[2]for q in truth_local],linestyle='--',label='Scoring truth Up');height.set(xlabel='Simulation time / s',ylabel='Ground-relative Up / m');height.legend()
fig.tight_layout();fig.savefig(b/'navigation-free-xyz.png',dpi=160);plt.close(fig)
truth_length=sum(math.dist(a,b)for a,b in zip(truth_local,truth_local[1:]));duration=truth[-1]['sim']-truth[0]['sim']
report={'scope':'Offline recorded-state and RPC scoring-truth geometry; no new flight',
 'planar_path':task['planar_path'],'path_messages':len(plans),'target_ground_relative_enu_m':target,
 'estimated_height_ground_relative_m':[min(q[2]for q in estimated),max(q[2]for q in estimated)],
 'maximum_planned_height_deviation_from_global_goal_line_m':height_deviation,
 'sampled_truth_trajectory_length_m':truth_length,'sampled_truth_duration_sim_s':duration,
 'sampled_truth_trajectory_average_m_s':truth_length/duration,
 'limitations':'Truth is scoring only. Sampled truth length excludes the unsampled start/end intervals and is not a continuous trajectory integral. Path height deviation alone does not certify every obstacle required vertical avoidance.'}
(b/'navigation-geometry.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
