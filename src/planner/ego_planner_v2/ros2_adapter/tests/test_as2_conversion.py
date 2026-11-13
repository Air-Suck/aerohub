"""Real generated ROS messages and baseline/repaired EGO output contracts."""
import json
import math
from pathlib import Path
import sys
import unittest
import copy
from array import array
import yaml
import numpy as np
from nav_msgs.msg import Odometry
from traj_utils.msg import PolyTraj
from uav_ego_planner_adapter.conversion import Polynomial,segment_for,world_odometry,set_ns,nanoseconds,equivalent_final_tail
from uav_planner_framework.follow_path_client import make_goal

project = Path(__file__).resolve().parents[5]

def message(row):
    msg = PolyTraj(drone_id=row['drone_id'],traj_id=row['id'],order=row['order'])
    set_ns(msg.start_time,round(row['start']*1e9))
    for field in ['duration','coef_x','coef_y','coef_z']: setattr(msg,field,[float(x) for x in row[field]])
    return msg

class Contracts(unittest.TestCase):
    def test_late_final_tail_requires_full_curve_equivalence(self):
        rows=json.loads((project/'src/planner/ego_planner_v2/ros2_adapter/tests/data/equivalent-final-polynomials.json').read_text())
        for index,vehicle in enumerate(['drone0','drone1']):
            old,new=rows[vehicle][-3:-1]
            def convert(row):
                msg=PolyTraj(drone_id=index,order=5);set_ns(msg.start_time,row['start_ns'])
                for field in ['duration','coef_x','coef_y','coef_z']:setattr(msg,field,row[field])
                return msg
            a=Polynomial(convert(old),index,.2415,.3675);msg=convert(new)
            b=Polynomial(msg,index,.2415,.3675);goal=[1.5,index*4+.2,2.]
            self.assertTrue(equivalent_final_tail(a,b,goal,b.start_ns+30_000_000,.0175))
            self.assertTrue(equivalent_final_tail(a,b,goal,max(a.end_ns,b.end_ns)+100_000_000,.0175))
            # A similar endpoint does not authorize discarding a changed path.
            for i in range(len(msg.duration)):msg.coef_x[6*i+5]+=.01
            changed=Polynomial(msg,index,.2415,.3675)
            self.assertTrue(changed.final(goal))
            self.assertFalse(equivalent_final_tail(a,changed,goal,b.start_ns+30_000_000,.0175))

    def test_body_twist_and_nonzero_origin(self):
        msg = Odometry(); msg.header.frame_id='drone1/odom';msg.child_frame_id='drone1/base_link'
        set_ns(msg.header.stamp,123456789012)
        msg.pose.pose.position.x=2.;msg.pose.pose.orientation.z=math.sqrt(.5);msg.pose.pose.orientation.w=math.sqrt(.5)
        msg.twist.twist.linear.x=1.;msg.twist.covariance[0]=2.;msg.twist.covariance[7]=3.
        out=world_odometry(msg,'drone1',[4.,5.,-2.])
        self.assertEqual(nanoseconds(out.header.stamp),123456789012)
        self.assertEqual([out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z],[7.,4.,2.])
        self.assertEqual(out.child_frame_id,'drone1/base_link')
        self.assertEqual(out.twist,msg.twist)
        msg.child_frame_id='unknown'
        with self.assertRaises(ValueError):world_odometry(msg,'drone1',[0.,0.,0.])

    def test_reject_unsafe_baseline_and_accept_repaired(self):
        baseline=project/'src/planner/ego_planner_v2/ros2_adapter/tests/data/unsafe-baseline.jsonl'
        old=next(json.loads(s) for s in baseline.read_text().splitlines() if json.loads(s)['kind']=='trajectory')
        with self.assertRaises(ValueError):Polynomial(message(old),0,.2415,.3675)
        repaired=project/'src/planner/ego_planner_v2/ros2_adapter/tests/data/repaired-free.jsonl'
        rows=[json.loads(s) for s in repaired.read_text().splitlines() if json.loads(s)['kind']=='trajectory']
        for row in rows: Polynomial(message(row),0,.2415,.3675)
        msg=message(rows[0]);msg.order=3
        with self.assertRaises(ValueError):Polynomial(msg,0,.2415,.3675)
        msg=message(rows[0]);msg.coef_x=array('f',[float('nan')]*len(msg.coef_x))
        with self.assertRaises(ValueError):Polynomial(msg,0,.2415,.3675)

    def test_epoch_pva_frames_and_sampling(self):
        path=project/'src/planner/ego_planner_v2/ros2_adapter/tests/data/repaired-free.jsonl'
        row=next(json.loads(s) for s in path.read_text().splitlines() if json.loads(s)['kind']=='trajectory')
        source=Polynomial(message(row),0,.2415,.3675)
        config=yaml.safe_load((project/'src/planner/ego_planner_v2/ros2_adapter/tests/data/dual.yaml').read_text())
        config['vehicles']=['drone1']
        segment=segment_for(config,{'drone1':source},{'drone1':[3,0,2]},source.start_ns+100_000_000)
        goal=make_goal(segment,segment.vehicle_trajectories[0],config['deployment']['drone1'],1,.2415)
        times=[nanoseconds(p.time_from_start) for p in goal.path.points]
        self.assertEqual(times[0],0);self.assertTrue(all(a<b for a,b in zip(times,times[1:])))
        self.assertEqual(nanoseconds(goal.path.valid_from),source.start_ns+500_000_000)
        for point in goal.path.points:
            p,v,a=source.sample(nanoseconds(goal.path.valid_from)+nanoseconds(point.time_from_start))
            actual=[point.position.x,point.position.y,point.position.z]
            np.testing.assert_allclose(actual,p-[0,4,0],atol=1e-12)
            np.testing.assert_allclose([point.velocity.x,point.velocity.y,point.velocity.z],v,atol=1e-12)
            np.testing.assert_allclose([point.acceleration.x,point.acceleration.y,point.acceleration.z],a,atol=1e-12)
        with self.assertRaises(ValueError):segment_for(config,{'drone1':source},{'drone1':[3,0,2]},source.end_ns)
        short=segment_for(config,{'drone1':source},{'drone1':[3,0,2]},source.end_ns-300_000_000)
        self.assertEqual(nanoseconds(short.valid_from),source.end_ns-100_000_000)
        self.assertEqual(nanoseconds(short.valid_from)+nanoseconds(short.vehicle_trajectories[0].points[-1].time_from_start),source.end_ns)
        padded=segment_for(config,{'drone1':source},{'drone1':[3,0,2]},source.end_ns-100_000_000)
        self.assertEqual(nanoseconds(padded.valid_from),source.end_ns-50_000_000)
        self.assertEqual(nanoseconds(padded.valid_from)+nanoseconds(padded.vehicle_trajectories[0].points[-1].time_from_start),source.end_ns+100_000_000)
        with self.assertRaises(ValueError):segment_for(config,{'drone1':source},{'drone1':[3,0,2]},source.end_ns-50_000_000)

    def test_common_final_epoch_with_shorter_vehicle_holding(self):
        path=project/'src/planner/ego_planner_v2/ros2_adapter/tests/data/repaired-free.jsonl'
        row=next(json.loads(s) for s in path.read_text().splitlines() if json.loads(s)['kind']=='trajectory')
        first=message(row);second=copy.deepcopy(first);second.drone_id=1
        set_ns(second.start_time,nanoseconds(first.start_time)+1_000_000_000)
        a=Polynomial(first,0,.2415,.3675);b=Polynomial(second,1,.2415,.3675)
        config=yaml.safe_load((project/'src/planner/ego_planner_v2/ros2_adapter/tests/data/dual.yaml').read_text())
        segment=segment_for(config,{'drone0':a,'drone1':b},{v:[3,0,2] for v in config['vehicles']},a.end_ns-200_000_000)
        self.assertTrue(segment.is_final)
        self.assertEqual(nanoseconds(segment.valid_from)+nanoseconds(segment.vehicle_trajectories[0].points[-1].time_from_start),b.end_ns)
        for p in segment.vehicle_trajectories[0].points:
            self.assertLess(math.dist([p.position.y,p.position.x,-p.position.z],[3,0,2]),1e-5)
            self.assertEqual([p.velocity.x,p.velocity.y,p.velocity.z],[0,0,0])

if __name__=='__main__':unittest.main()
