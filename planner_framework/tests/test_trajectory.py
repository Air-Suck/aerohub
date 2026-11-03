import math
import unittest

from uav_planner_framework import (
    ConfigError, OutputGateway, TimedTrajectorySegment, TrajectoryPoint, Vector3, VehicleTrajectory,
)


def segment(generated_at=1.0, vehicle_ids=("uav1", "uav2")):
    trajectories = tuple(
        VehicleTrajectory(
            vehicle_id,
            frozenset(),
            (TrajectoryPoint(0.0, Vector3(0.0, 0.0, -1.0)),
             TrajectoryPoint(2.0, Vector3(2.0, 0.0, -1.0))),
        )
        for vehicle_id in vehicle_ids
    )
    return TimedTrajectorySegment(1, "task-1", generated_at, 10.0, True, trajectories)


class GatewayTest(unittest.TestCase):
    def setUp(self):
        self.gateway = OutputGateway(
            "task-1", {"uav1": Vector3(0.0, 0.0, 0.0), "uav2": Vector3(1.0, 0.0, 0.0)}
        )

    def test_accept_sample_and_origin_conversion(self):
        self.assertTrue(self.gateway.accept(segment(), now=9.0))
        initial = self.gateway.sample("uav1", now=9.0)
        self.assertAlmostEqual(initial.position.x, 0.0)
        setpoint = self.gateway.sample("uav2", now=11.0)
        self.assertAlmostEqual(setpoint.position.x, 0.0)
        self.assertTrue(math.isnan(setpoint.velocity.x))
        self.assertTrue(math.isnan(setpoint.yaw))

    def test_duplicate_is_ignored_and_older_is_rejected(self):
        self.gateway.accept(segment(2.0), now=9.0)
        self.assertFalse(self.gateway.accept(segment(2.0), now=9.0))
        with self.assertRaisesRegex(ConfigError, "out-of-order"):
            self.gateway.accept(segment(1.0), now=9.0)

    def test_incomplete_fleet_is_rejected(self):
        with self.assertRaisesRegex(ConfigError, "vehicle set"):
            self.gateway.accept(segment(vehicle_ids=("uav1",)), now=9.0)

    def test_expired_segment_is_rejected(self):
        with self.assertRaisesRegex(ConfigError, "no executable coverage"):
            self.gateway.accept(segment(), now=12.0)

    def test_non_final_segment_times_out_after_coverage_gap(self):
        value = segment()
        value = TimedTrajectorySegment(
            value.interface_version, value.task_id, value.generated_at, value.valid_from,
            False, value.vehicle_trajectories,
        )
        self.gateway.accept(value, now=9.0)
        self.assertFalse(self.gateway.trajectory_timed_out(12.5, 1.0))
        self.assertTrue(self.gateway.trajectory_timed_out(13.1, 1.0))
        self.gateway.clear()
        self.assertIsNone(self.gateway.segment)

    def test_waiting_for_first_segment_times_out(self):
        self.assertFalse(self.gateway.trajectory_timed_out(4.9, 5.0, waiting_since=0.0))
        self.assertTrue(self.gateway.trajectory_timed_out(5.1, 5.0, waiting_since=0.0))

    def test_yaw_interpolation_uses_shortest_rotation(self):
        trajectory = VehicleTrajectory(
            "uav1",
            frozenset({"yaw"}),
            (
                TrajectoryPoint(0.0, Vector3(0.0, 0.0, 0.0), yaw=3.0),
                TrajectoryPoint(2.0, Vector3(0.0, 0.0, 0.0), yaw=-3.0),
            ),
        )
        value = TimedTrajectorySegment(1, "task-1", 1.0, 10.0, True, (trajectory,))
        gateway = OutputGateway("task-1", {"uav1": Vector3(0.0, 0.0, 0.0)})
        gateway.accept(value, now=9.0)
        self.assertAlmostEqual(abs(gateway.sample("uav1", now=11.0).yaw), math.pi)


if __name__ == "__main__":
    unittest.main()
