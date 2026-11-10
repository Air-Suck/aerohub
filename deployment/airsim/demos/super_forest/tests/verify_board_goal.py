"""Validate ROS runtime goal configuration without arming or execution."""
import unittest
import rclpy
from rclpy.parameter import Parameter
from uav_super_adapter.node import SuperAdapter


class GoalParametersTest(unittest.TestCase):
    def test_goal_can_only_change_while_inactive_and_valid(self):
        rclpy.init();node=SuperAdapter()
        try:
            parameter=Parameter.from_parameter_msg(Parameter('goal',value=[1.,2.,3.]).to_parameter_msg())
            result=node.set_parameters_atomically([parameter])
            self.assertTrue(result.successful);self.assertEqual(node.goal,[1.,2.,3.])
            node.enabled=True
            result=node.set_parameters_atomically([Parameter('goal',value=[4.,5.,6.])])
            self.assertFalse(result.successful);self.assertEqual(node.goal,[1.,2.,3.])
            node.enabled=False
            result=node.set_parameters_atomically([Parameter('goal',value=[float('nan'),0.,2.])])
            self.assertFalse(result.successful);self.assertEqual(node.goal,[1.,2.,3.])
        finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':unittest.main()
