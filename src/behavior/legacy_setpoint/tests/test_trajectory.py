import math
import unittest
from types import SimpleNamespace as S

from as2_msgs.msg import ControlMode, ControllerInfo, TrajectorySetpoints
from geometry_msgs.msg import PoseStamped
from mars_quadrotor_msgs.msg import PositionCommand
from uav_as2_setpoint_adapter.super_node import convert_command, SuperAdapter
from uav_as2_setpoint_adapter.node import As2SetpointAdapter

class AdapterTest(unittest.TestCase):
    def command(self):
        m = PositionCommand()
        m.header.frame_id = 'drone0/odom'
        m.header.stamp.sec = 10
        m.position.x,m.position.y,m.position.z=1.,2.,3.
        m.velocity.x,m.velocity.y,m.velocity.z=4.,5.,6.
        m.acceleration.x,m.acceleration.y,m.acceleration.z=7.,8.,9.
        m.trajectory_flag = 1
        m.yaw = 0.75
        return m

    def test_primary_backup_and_yaw(self):
        for flag in (1,2):
            m=self.command();m.trajectory_flag=flag
            o=convert_command(m,'drone0/odom',0.5)
            self.assertEqual(o.header,m.header)
            self.assertEqual(o.setpoints[0].position.x,1.)
            self.assertEqual(o.setpoints[0].twist.y,5.)
            self.assertEqual(o.setpoints[0].acceleration.z,9.)
            self.assertAlmostEqual(o.setpoints[0].yaw_angle,0.5)
            self.assertAlmostEqual(convert_command(m,'drone0/odom',0.5,True).setpoints[0].yaw_angle,.75)

    def test_rejections(self):
        m=self.command()
        for flag in (0,3,4,5):
            m.trajectory_flag=flag
            with self.assertRaises(ValueError):convert_command(m,'drone0/odom',0.)
        m=self.command()
        with self.assertRaises(ValueError):convert_command(m,'map',0.)
        m.acceleration.x=math.inf
        with self.assertRaises(ValueError):convert_command(m,'drone0/odom',0.)
        m=self.command();m.yaw=math.nan
        self.assertEqual(convert_command(m,'drone0/odom',.5).setpoints[0].yaw_angle,.5)
        with self.assertRaises(ValueError):convert_command(m,'drone0/odom',.5,True)

    def test_fleet_position_only(self):
        published=[]
        n=S(latest_setpoints=None, task_id='task', vehicles=['v'],
            setpoints_seen=0, published_counts={'v':0},
            mode_ready={'v':True}, held_yaw={'v':.25}, current_yaw={'v':.75},
            config={'deployment':{'v':{'namespace':'drone0'}}},
            trajectory_publishers={'v':S(publish=published.append)})
        p=lambda x,y,z:S(x=x,y=y,z=z)
        msg=S(task_id='task',sampled_at=self.command().header.stamp,setpoints=[
            S(vehicle_id='v',position=p(1.,2.,-3.),velocity=p(math.nan,math.nan,math.nan),
              acceleration=p(math.nan,math.nan,math.nan),yaw=math.nan)])
        n._publish_if_ready=lambda: As2SetpointAdapter._publish_if_ready(n)
        As2SetpointAdapter._on_setpoints(n,msg)
        self.assertEqual(len(published),1)
        o=published[0];self.assertEqual(o.header.stamp,msg.sampled_at)
        self.assertEqual((o.setpoints[0].position.x,o.setpoints[0].position.y,o.setpoints[0].position.z),(2.,1.,3.))
        self.assertTrue(math.isnan(o.setpoints[0].twist.x))
        self.assertTrue(math.isnan(o.setpoints[0].acceleration.z))
        self.assertEqual(o.setpoints[0].yaw_angle,.25)
        self.assertIsNone(n.latest_setpoints)

    def test_no_automatic_super_resume(self):
        n=S(ready=True,enabled=True)
        msg=ControllerInfo();msg.input_control_mode.control_mode=ControlMode.HOVER
        SuperAdapter.on_mode(n,msg);self.assertFalse(n.enabled)
        msg.input_control_mode.control_mode=ControlMode.TRAJECTORY
        msg.input_control_mode.yaw_mode=ControlMode.YAW_ANGLE
        SuperAdapter.on_mode(n,msg);self.assertTrue(n.ready);self.assertFalse(n.enabled)

class MissionStartupTest(unittest.TestCase):
    def test_construct_local_odometry_consumers(self):
        import rclpy
        from nav_msgs.msg import Odometry
        from uav_as2_setpoint_adapter.super_mission import SuperMission
        rclpy.init()
        mission = SuperMission()
        adapter = SuperAdapter()
        try:
            odom = Odometry()
            odom.header.frame_id = 'drone0/odom'
            odom.pose.pose.orientation.w = 1.0
            odom.pose.pose.position.z = 3.8
            mission.on_odometry(odom)
            adapter.on_pose(odom)
            self.assertEqual(mission.pose.pose.position.z, 3.8)
            self.assertEqual(mission.pose.header.frame_id, 'drone0/odom')
            self.assertEqual(adapter.current_yaw, 0.0)
        finally:
            mission.destroy_node()
            adapter.destroy_node()
            rclpy.shutdown()

if __name__=='__main__':unittest.main()
