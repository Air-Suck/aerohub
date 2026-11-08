"""ROS message boundary regression; run in the Humble runtime image."""
import unittest
from unittest.mock import Mock
from concurrent.futures import Future

try:
    from uav_planner_interfaces.msg import TimedTrajectorySegment, VehicleTrajectory, TimedTrajectoryPoint
    from uav_planner_framework.follow_path_client import make_goal, FleetFollowPathClient
except ImportError:
    TimedTrajectorySegment = None


@unittest.skipIf(TimedTrajectorySegment is None, "requires generated ROS messages")
class FollowPathConversionTest(unittest.TestCase):
    def test_one_rejection_stops_fleet_and_late_acceptance_is_canceled(self):
        coordinator = FleetFollowPathClient.__new__(FleetFollowPathClient)
        coordinator.closed = coordinator.failed = coordinator.cancel_started = False
        coordinator.stop_feedback = {"a", "b"}
        coordinator.stop_complete = False
        coordinator.generation = 0
        coordinator.pending = {"a", "b"}
        coordinator.results = set()
        coordinator.actions = {"a": Mock(), "b": Mock()}
        accepted = Mock(accepted=True)
        coordinator.handles = {"a": accepted}
        coordinator.stoppers = {"a": Mock(), "b": Mock()}
        coordinator.stop_futures = []
        coordinator.on_failure = Mock()
        for stopper in coordinator.stoppers.values():
            stopper.service_is_ready.return_value = True
            future = Future(); future.set_result(Mock(success=True))
            stopper.call_async.return_value = future
        rejected = Future(); rejected.set_result(Mock(accepted=False))
        coordinator._accepted("b", 0, rejected)
        accepted.cancel_goal_async.assert_called_once()
        for stopper in coordinator.stoppers.values():
            stopper.call_async.assert_called_once()
        coordinator.on_failure.assert_called_once()
        self.assertTrue(coordinator.quiescent)
        late = Mock(accepted=True)
        future = Future(); future.set_result(late)
        coordinator._accepted("b", 0, future)
        late.cancel_goal_async.assert_called_once()

    def test_stop_rejection_and_missing_feedback_block_quiescence(self):
        coordinator = FleetFollowPathClient.__new__(FleetFollowPathClient)
        coordinator.cancel_started = True
        coordinator.stop_complete = False
        coordinator.actions = {"a": Mock()}
        coordinator.stop_feedback = {"a"}
        future = Future()
        future.set_result(Mock(success=False))
        coordinator.stop_futures = [future]
        self.assertFalse(coordinator.quiescent)
        accepted = Future()
        accepted.set_result(Mock(success=True))
        coordinator.stop_futures = [accepted]
        coordinator.stop_feedback.clear()
        self.assertFalse(coordinator.quiescent)
        coordinator.stop_feedback.add("a")
        self.assertTrue(coordinator.quiescent)
        coordinator.stop_feedback.clear()
        self.assertTrue(coordinator.quiescent)  # Landing cannot undo confirmed stop.

    def test_preserves_time_derivatives_identity_and_source(self):
        segment = TimedTrajectorySegment()
        segment.interface_version = 1
        segment.task_id = "fleet"
        segment.valid_from.sec = 123
        segment.valid_from.nanosec = 456
        segment.generated_at.sec = 122
        segment.is_final = True
        vehicle = VehicleTrajectory()
        vehicle.vehicle_id = "drone2"
        vehicle.valid_fields = 7
        point = TimedTrajectoryPoint()
        point.time_from_start.sec = 3
        point.position.x, point.position.y, point.position.z = 11., 22., -3.
        point.velocity.x, point.velocity.y, point.velocity.z = 1., 2., -1.
        point.acceleration.x, point.acceleration.y, point.acceleration.z = 3., 4., -2.
        point.yaw = 0.
        vehicle.points = [point]
        goal = make_goal(segment, vehicle, {
            "namespace": "/drone2", "px4_origin_in_fleet_ned": [10., 20., 0.]
        }, 8, 2.)
        p = goal.path
        self.assertEqual((p.task_id, p.vehicle_id, p.revision), ("fleet", "drone2", 8))
        self.assertEqual(p.frame_id, "drone2/odom")
        self.assertEqual(p.valid_from, segment.valid_from)
        self.assertEqual(p.points[0].time_from_start.sec, 3)
        self.assertEqual((p.points[0].position.x, p.points[0].position.y, p.points[0].position.z), (2., 1., 3.))
        self.assertEqual((p.points[0].velocity.x, p.points[0].velocity.y, p.points[0].velocity.z), (2., 1., 1.))
        self.assertEqual((p.points[0].acceleration.x, p.points[0].acceleration.y, p.points[0].acceleration.z), (4., 3., 2.))
        self.assertAlmostEqual(p.points[0].yaw, 1.5707963267948966)
        self.assertEqual(point.position.x, 11.)
        self.assertEqual(point.velocity.x, 1.)


@unittest.skipIf(TimedTrajectorySegment is None, "requires generated ROS messages")
class ExecutionFeedbackTest(unittest.TestCase):
    def test_settling_uses_active_revision_not_latest_pending_plan(self):
        coordinator=FleetFollowPathClient.__new__(FleetFollowPathClient)
        coordinator.closed=coordinator.failed=False
        coordinator.generation=0
        coordinator.plans={("a",1):(False,1.),("a",2):(True,3.),("a",3):(False,10.)}
        coordinator.feedbacks={}
        feedback=Mock(revision=1,elapsed=2.,waiting_for_start=False)
        coordinator._feedback("a",0,Mock(feedback=feedback))
        self.assertEqual(coordinator.settling_vehicles,set())
        feedback=Mock(revision=2,elapsed=2.9,waiting_for_start=False)
        coordinator._feedback("a",0,Mock(feedback=feedback))
        self.assertEqual(coordinator.settling_vehicles,set())
        feedback.elapsed=3.
        self.assertEqual(coordinator.settling_vehicles,{"a"})
        coordinator._feedback("a",0,Mock(feedback=Mock(revision=1,elapsed=10.,waiting_for_start=False)))
        self.assertEqual(coordinator.settling_vehicles,{"a"})
        coordinator.closed=True
        self.assertEqual(coordinator.settling_vehicles,set())
