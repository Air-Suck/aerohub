"""Actual ROS Adapter construction; executor handoff gating without flight."""
import time
import unittest
from types import SimpleNamespace
import rclpy
from as2_msgs.msg import TrajectorySetpoints,TrajectoryPoint
from mars_quadrotor_msgs.msg import PolynomialTrajectory
from uav_super_adapter.node import SuperAdapter


class HandoffTest(unittest.TestCase):
    def test_update_waits_for_first_current_trajectory_reference(self):
        rclpy.init();node=SuperAdapter();submitted=[]
        try:
            node.enabled=True;node.last_wall=time.monotonic();node.first_start=node.now_seconds()
            node.client=SimpleNamespace(revision=1,complete=False,ready=True,pending=set(),
                settling_vehicles=set(),quiescent=False,cancel_started=False,
                tick=lambda _:None,submit=submitted.append)
            message=PolynomialTrajectory();message.header.stamp=node.get_clock().now().to_msg()
            node.queued=message;node.convert=lambda _:object()
            node.tick()
            self.assertEqual(submitted,[], 'Rolling update raced the initial motion-mode handoff')
            reference=TrajectorySetpoints();reference.header.frame_id=node.frame;reference.setpoints=[TrajectoryPoint()]
            reference.header.stamp.sec=int(node.first_start)-1
            node.on_reference(reference);node.tick()
            self.assertEqual(submitted,[],'Old references must not release the current handoff')
            reference.header.stamp=node.get_clock().now().to_msg()
            node.on_reference(reference);node.tick()
            self.assertEqual(len(submitted),1)
        finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':unittest.main()
