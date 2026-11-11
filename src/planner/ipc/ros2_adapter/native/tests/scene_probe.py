"""Read a real registered scan to preflight candidate forest goals, without simulation."""
import argparse,json,math,sqlite3
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from uav_follow_reference_acro.msg import ControlSession
p=argparse.ArgumentParser();p.add_argument('evidence',type=Path);a=p.parse_args()
c=sqlite3.connect('file:'+str(next((a.evidence/'rosbag').glob('*.db3')))+'?mode=ro',uri=True)
active=None
for stamp,data in c.execute("select m.timestamp,m.data from messages m join topics t on t.id=m.topic_id where t.name='/drone0/external_acro/session' order by m.timestamp"):
 if deserialize_message(data,ControlSession).active:active=stamp;break
if active is None:raise RuntimeError('no active session')
r=c.execute("select m.data from messages m join topics t on t.id=m.topic_id where t.name='/drone0/navigation/points' and m.timestamp<=? order by m.timestamp desc limit 1",(active,)).fetchone()
if not r:raise RuntimeError('raw registered scan not recorded in this run')
m=deserialize_message(r[0],PointCloud2)
if m.header.frame_id!='earth' or m.is_bigendian:raise RuntimeError('unexpected scan frame/layout')
offsets={f.name:f.offset for f in m.fields}
d=np.ndarray((m.width*m.height,),dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[offsets[k]for k in ('x','y','z')],'itemsize':m.point_step}),buffer=m.data)
points=np.column_stack([d[k]for k in ('x','y','z')]);points=points[np.isfinite(points).all(axis=1)]
origin=np.asarray(json.loads((a.evidence/'result.json').read_text())['target']);relative=points-origin
candidates=[[-3.2,19.74,0.]]+[[20*math.cos(i*math.pi/8),20*math.sin(i*math.pi/8),0.]for i in range(16)]
result=[]
for delta in candidates:
 delta=np.asarray(delta);projection=(relative[:,0]*delta[0]+relative[:,1]*delta[1])/(delta@delta)
 middle=relative[(projection>.1)&(projection<.9)];t=projection[(projection>.1)&(projection<.9)]
 distance=np.linalg.norm(middle-t[:,None]*delta,axis=1)
 local=delta*min(1,9/np.max(np.abs(delta[:2])))
 result.append({'offset_enu_m':delta.tolist(),'direct_line_min_point_distance_m':float(distance.min()) if len(distance) else None,'goal_point_clearance_m':float(np.linalg.norm(relative-delta,axis=1).min()),'first_local_goal_clearance_m':float(np.linalg.norm(relative-local,axis=1).min())})
print(json.dumps({'scope':'single real LiDAR scan, sparse visibility; not a route guarantee','source':str(a.evidence),'frame':m.header.frame_id,'origin':origin.tolist(),'candidates':result},indent=2))
