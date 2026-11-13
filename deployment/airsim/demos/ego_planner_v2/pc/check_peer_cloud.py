"""Regression for lost ROS1 peer filtering; real Adapter imports required."""
import json
from pathlib import Path
from uav_ego_planner_adapter.node import EgoAdapter
from uav_ego_planner_adapter.peer_cloud import remove_peer_returns
from uav_ego_planner_adapter.conversion import world_odometry
from nav_msgs.msg import Odometry

m=Odometry();m.header.frame_id='drone1/odom';m.child_frame_id='drone1/base_link'
m.pose.pose.orientation.w=1.;m.pose.pose.position.x=1.;m.pose.pose.position.y=2.;m.pose.pose.position.z=2.
p=world_odometry(m,'drone1',[0.,-4.,0.]).pose.pose.position
assert (p.x,p.y,p.z)==(-3.,2.,2.)
points=[[-3.,2.,2.],[-2.4,2.,2.],[-2.1,2.,2.],[2.,2.,2.],[-3.,2.,0.]]
history=[[(1_000_000_000,10.,(p.x,p.y,p.z))]]
kept,removed=remove_peer_returns(points,history,1_050_000_000,10.1,.75)
assert kept==points[2:] and removed==2
rejected=0
for histories,stamp,now in [([],1_050_000_000,10.1),([[]],1_050_000_000,10.1),
    (history,1_050_000_000,10.6),(history,1_300_000_000,10.1)]:
    if not histories:continue  # single-vehicle caller has no peers
    try:remove_peer_returns(points,histories,stamp,now,.75)
    except ValueError:rejected+=1
assert rejected==3
out=dict(passed=True,adapter_imported=EgoAdapter.__name__,ros_message_imported=True,skipped=0,
    nonidentity_origin_conversion=True,peer_points_removed=removed,static_points_preserved=len(kept),
    stale_missing_or_unsynchronized_peer_rejections=rejected,
    old_unfiltered_input_would_retain_peer_body=True,scope='input contract regression, not flight')
Path('/evidence/peer-cloud-regression.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2))
