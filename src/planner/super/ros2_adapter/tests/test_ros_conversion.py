import unittest
from unittest.mock import Mock
try:
    from mars_quadrotor_msgs.msg import PolynomialTrajectory
    from uav_super_adapter.node import SuperAdapter
except ImportError:
    PolynomialTrajectory=None


@unittest.skipIf(PolynomialTrajectory is None,'requires generated ROS messages')
class ConversionTest(unittest.TestCase):
    def test_preserves_source_epoch_full_suffix_and_pva(self):
        msg=PolynomialTrajectory()
        msg.header.stamp.sec=100
        msg.start_wt_pos=99.
        msg.piece_num_pos=1;msg.order_pos=3
        msg.time_pos=[3.]
        msg.coef_pos_x=[0.,0.,1.,0.]
        msg.coef_pos_y=[0.,0.,0.,0.]
        msg.coef_pos_z=[0.,0.,0.,2.]
        node=SuperAdapter.__new__(SuperAdapter)
        node.now_seconds=lambda:100.
        node.get_parameter=lambda _:Mock(value=.1)
        node.goal=[3.,0.,2.];node.task='test';node.vehicle='drone0'
        segment=node.convert(msg)
        self.assertEqual(segment.generated_at.sec,100)
        self.assertAlmostEqual(segment.valid_from.sec+segment.valid_from.nanosec/1e9,100.1)
        self.assertFalse(segment.is_final) # Endpoint is moving, not an arrival.
        item=segment.vehicle_trajectories[0]
        self.assertEqual(item.valid_fields,3)
        self.assertAlmostEqual(item.points[0].position.y,1.1)
        self.assertAlmostEqual(item.points[-1].position.y,3.)
        self.assertAlmostEqual(item.points[0].velocity.y,1.)
        self.assertAlmostEqual(item.points[-1].time_from_start.sec+item.points[-1].time_from_start.nanosec/1e9,1.9)
        self.assertEqual(item.points[0].position.z,-2.)

    def test_task_tolerance_recognizes_stationary_final_segment(self):
        msg=PolynomialTrajectory()
        msg.header.stamp.sec=100;msg.start_wt_pos=99.
        msg.piece_num_pos=1;msg.order_pos=3;msg.time_pos=[3.]
        msg.coef_pos_x=[0.,0.,0.,3.];msg.coef_pos_y=[0.]*4;msg.coef_pos_z=[0.,0.,0.,2.]
        node=SuperAdapter.__new__(SuperAdapter)
        node.now_seconds=lambda:100.
        node.get_parameter=lambda k:Mock(value=.1 if k=='lead_time' else 1.5)
        node.goal=[3.8,0.,2.];node.task='test';node.vehicle='drone0'
        self.assertTrue(node.convert(msg).is_final)

    def test_expired_update_cannot_create_a_new_execution_epoch(self):
        from uav_super_adapter.polynomial import InsufficientCoverage
        msg=PolynomialTrajectory()
        msg.header.stamp.sec=100;msg.start_wt_pos=99.
        msg.piece_num_pos=1;msg.order_pos=3;msg.time_pos=[1.]
        msg.coef_pos_x=msg.coef_pos_y=msg.coef_pos_z=[0.]*4
        node=SuperAdapter.__new__(SuperAdapter)
        node.now_seconds=lambda:100.
        node.get_parameter=lambda _:Mock(value=.1)
        node.goal=[0.,0.,0.];node.task='test';node.vehicle='drone0'
        with self.assertRaises(InsufficientCoverage):node.convert(msg)

    def test_heartbeat_and_repeated_command_do_not_refresh_execution(self):
        from mars_quadrotor_msgs.msg import PositionCommand
        node=SuperAdapter.__new__(SuperAdapter)
        node.enabled=True;node.frame='drone0/odom';node.last_plan=99.
        node.queued=None;node.now_seconds=lambda:100.
        node.last_command=100.;node.last_wall=9.;node.flags={};node.fail=Mock()
        heartbeat=PolynomialTrajectory(type=PolynomialTrajectory.HEART_BEAT)
        node.on_plan(heartbeat)
        self.assertIsNone(node.queued)
        self.assertEqual(node.last_plan,99.)
        command=PositionCommand(trajectory_flag=2)
        command.header.frame_id=node.frame;command.header.stamp.sec=100
        node.on_command(command)
        self.assertEqual(node.last_wall,9.)
        node.last_command=99.9
        node.on_command(command)
        self.assertEqual(node.flags,{'2':1})
        self.assertGreater(node.last_wall,9.)
        command.header.stamp.nanosec=1000000;command.trajectory_flag=0
        node.on_command(command)
        node.fail.assert_called_once()
        node.fail.reset_mock()
        command.trajectory_flag=2;command.header.stamp.nanosec=2000000
        command.position.x=float('nan')
        node.on_command(command)
        node.fail.assert_called_once()
        self.assertEqual(node.last_command,100.)

    def test_late_update_keeps_existing_plan_without_renewal(self):
        from unittest.mock import patch
        from uav_super_adapter.polynomial import InsufficientCoverage
        node=SuperAdapter.__new__(SuperAdapter)
        node.client=Mock(complete=False,ready=True,pending=set(),revision=5,cancel_started=False,settling_vehicles=set())
        node.enabled=True;node.last_wall=9.8;node.started=1.;node.now_seconds=lambda:100.
        node.queued=PolynomialTrajectory();node.queued.header.stamp.sec=100
        node.late_updates=0;node.last_curve={};node.flags={};node.error='';node.task='test'
        node.publisher=Mock();node.fail=Mock();node.vehicle='drone0'
        node.convert=Mock(side_effect=InsufficientCoverage('late'))
        with patch('uav_super_adapter.node.time.monotonic',return_value=10.):node.tick()
        self.assertTrue(node.enabled)
        self.assertEqual(node.late_updates,1)
        self.assertEqual(node.last_wall,9.8)
        node.client.submit.assert_not_called()
        node.fail.assert_not_called()
