import unittest

from uav_planner_framework import FleetCompletion, Vector3


class FleetCompletionTest(unittest.TestCase):
    def setUp(self):
        self.monitor = FleetCompletion(
            {"drone0": Vector3(1.0, 2.0, -1.0), "drone1": Vector3(3.0, 4.0, -1.0)},
            position_tolerance=0.5,
            hold_duration=1.0,
        )

    def test_all_vehicles_must_hold_for_full_duration(self):
        positions = {
            "drone0": Vector3(1.1, 2.0, -1.0),
            "drone1": Vector3(3.0, 4.2, -1.0),
        }
        self.assertFalse(self.monitor.update(positions, 10.0))
        self.assertFalse(self.monitor.update(positions, 10.9))
        self.assertTrue(self.monitor.update(positions, 11.0))

    def test_leaving_tolerance_resets_hold(self):
        inside = {
            "drone0": Vector3(1.0, 2.0, -1.0),
            "drone1": Vector3(3.0, 4.0, -1.0),
        }
        outside = dict(inside, drone1=Vector3(4.0, 4.0, -1.0))
        self.assertFalse(self.monitor.update(inside, 1.0))
        self.assertFalse(self.monitor.update(outside, 1.8))
        self.assertFalse(self.monitor.update(inside, 2.0))
        self.assertTrue(self.monitor.update(inside, 3.0))

    def test_missing_vehicle_cannot_complete(self):
        self.assertFalse(self.monitor.update({"drone0": Vector3(1.0, 2.0, -1.0)}, 5.0))


if __name__ == "__main__":
    unittest.main()
