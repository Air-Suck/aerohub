"""Read-only sample of real navigation points for obstacle-test selection."""
import json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2

rclpy.init();node=Node('ego_obstacle_survey');clouds=[];poses=[];frames=[]
def cloud(m):
    points=np.asarray([[float(v) for v in p] for p in point_cloud2.read_points(m,field_names=('x','y','z'),skip_nans=True)],dtype=float)
    clouds.append(points);frames.append(m.header.frame_id)
def odom(m):
    p=m.pose.pose.position;poses.append([p.x,p.y,p.z])
subs=[node.create_subscription(PointCloud2,'/drone0/navigation/points',cloud,qos_profile_sensor_data),
      node.create_subscription(Odometry,'/drone0/navigation/odom',odom,qos_profile_sensor_data)]
deadline=time.monotonic()+6
while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
assert len(clouds)>=10 and poses and set(frames)=={'drone0/odom'},'missing or invalid real input'
points=np.concatenate(clouds);_,ids=np.unique(np.round(points/.1).astype(int),axis=0,return_index=True)
points=points[ids];p=np.median(poses,axis=0)
out=Path('/evidence');np.save(out/'points.npy',points)
report={'frames':len(clouds),'points':len(points),'position':p.tolist(),'coordinate_frame':'drone0/odom'}
for low,high in [(.5,1.5),(1.5,2.5),(2.5,3.5)]:
    band=points[(points[:,2]>=p[2]+low)&(points[:,2]<=p[2]+high)]
    cells={tuple(x) for x in np.floor(band[:,:2]/.35).astype(int)};clusters=[]
    while cells:
        todo=[cells.pop()];group=[]
        while todo:
            x,y=todo.pop();group.append((x,y))
            for dx in [-1,0,1]:
                for dy in [-1,0,1]:
                    key=(x+dx,y+dy)
                    if key in cells:cells.remove(key);todo.append(key)
        if len(group)>=2:
            a=np.array(group)*.35
            clusters.append({'cells':len(group),'center':np.mean(a,axis=0).tolist(),'min':a.min(axis=0).tolist(),'max':(a.max(axis=0)+.35).tolist(),'distance':float(np.linalg.norm(np.mean(a,axis=0)-p[:2]))})
    report[f'height_{low}_{high}']=sorted(clusters,key=lambda v:v['distance'])[:12]
(out/'survey.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
node.destroy_node();rclpy.shutdown()
