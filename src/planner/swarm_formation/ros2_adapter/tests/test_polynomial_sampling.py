"""Native descending coefficients -> one-time PVA sample, including hold and joins."""
import importlib.util
from pathlib import Path
import unittest
from rclpy.time import Time
from rclpy.clock import ClockType
from traj_utils.msg import PolyTraj

path=Path(__file__).resolve().parents[2]/'humble/plan_manage/scripts/ros2_proxy.py'
spec=importlib.util.spec_from_file_location('swarm_proxy_under_test',path)
proxy=importlib.util.module_from_spec(spec);spec.loader.exec_module(proxy)


class PolynomialSamplingTest(unittest.TestCase):
    def message(self):
        p=PolyTraj();p.order=5;p.start_time=Time(seconds=100,clock_type=ClockType.ROS_TIME).to_msg()
        p.duration=[1.0];p.coef_x=[0.,0.,1.,2.,3.,4.];p.coef_y=[0.,0.,0.,0.,0.,10.];p.coef_z=[0.,0.,0.,0.,0.,3.]
        return p

    def test_analytic_derivatives(self):
        self.assertEqual(proxy.evaluate([0,0,1,2,3,4],0,5,2),26)
        self.assertEqual(proxy.evaluate([0,0,1,2,3,4],0,5,2,1),23)
        self.assertEqual(proxy.evaluate([0,0,1,2,3,4],0,5,2,2),16)

    def test_terminal_hold_has_zero_derivatives(self):
        msg=self.message();now=Time(seconds=100.5,clock_type=ClockType.ROS_TIME)
        positions=proxy.sample(msg,now,.25,1)
        velocities=proxy.sample(msg,now,.25,1,1)
        accelerations=proxy.sample(msg,now,.25,1,2)
        self.assertEqual(velocities[0],[5.75,0.,0.])
        self.assertEqual(accelerations[0],[7.,0.,0.])
        self.assertEqual(positions[-1],[10.,10.,3.])
        self.assertEqual(velocities[-1],[0.,0.,0.]);self.assertEqual(accelerations[-1],[0.,0.,0.])

    def test_piece_boundary_uses_correct_coefficient_block(self):
        msg=self.message();msg.duration=[1.,1.]
        msg.coef_x = list(msg.coef_x) + [0.,0.,1.,5.,10.,10.];msg.coef_y = list(msg.coef_y) + [0.,0.,0.,0.,0.,10.];msg.coef_z = list(msg.coef_z) + [0.,0.,0.,0.,0.,3.]
        now=Time(seconds=101,clock_type=ClockType.ROS_TIME)
        self.assertAlmostEqual(proxy.sample(msg,now,.1,.2)[0][0],10)
        self.assertAlmostEqual(proxy.sample(msg,now,.1,.2,1)[1][0],11.03)
        self.assertAlmostEqual(proxy.sample(msg,now,.1,.2,2)[1][0],10.6)


if __name__=='__main__':unittest.main()
