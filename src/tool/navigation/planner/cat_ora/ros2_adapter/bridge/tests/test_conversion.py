import unittest

from uav_cat_ora_adapter.node import (
    enu_to_ned,
    local_enu_to_fleet_enu,
    ned_to_enu,
    trajectories_reach_requested_goals,
)


class CoordinateConversionTest(unittest.TestCase):
    def test_ned_enu_round_trip(self):
        ned = {"x": 1.0, "y": 2.0, "z": -3.0}
        enu = ned_to_enu(ned)
        self.assertEqual(enu, [2.0, 1.0, 3.0])
        self.assertEqual(enu_to_ned(enu), (1.0, 2.0, -3.0))

    def test_local_pose_uses_vehicle_fleet_origin(self):
        self.assertEqual(
            local_enu_to_fleet_enu([0.5, 1.0, 2.0], [-3.0, 4.0, -0.2]),
            [4.5, -2.0, 2.2],
        )

    def test_reassigned_goal_is_rejected(self):
        goals = [[0.0, 0.0, 1.0], [2.0, 0.0, 1.0]]
        self.assertTrue(trajectories_reach_requested_goals(
            [[[1.0, 0.0, 1.0], goals[0]], [[1.0, 0.0, 1.0], goals[1]]], goals
        ))
        self.assertFalse(trajectories_reach_requested_goals(
            [[[1.0, 0.0, 1.0], goals[1]], [[1.0, 0.0, 1.0], goals[0]]], goals
        ))


if __name__ == "__main__":
    unittest.main()
