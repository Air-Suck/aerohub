import unittest

from uav_swarm_formation_adapter.node import (
    formation_center,
    local_enu_to_fleet_enu,
    validate_trajectory_starts,
)


class ConversionTest(unittest.TestCase):
    def test_local_enu_to_fleet_enu(self):
        self.assertEqual(
            local_enu_to_fleet_enu([1.0, 2.0, 3.0], [-4.0, -10.0, 0.0]),
            [-9.0, -2.0, 3.0],
        )

    def test_center_from_configured_shape(self):
        config = {
            "vehicles": [f"drone{i}" for i in range(7)],
            "targets": {
                "drone0": {"x": 0.0, "y": -4.0, "z": -1.5},
                "drone1": {"x": -1.0, "y": -2.2679, "z": -1.5},
                "drone2": {"x": -2.0, "y": -4.0, "z": -1.5},
                "drone3": {"x": -1.0, "y": -5.7321, "z": -1.5},
                "drone4": {"x": 1.0, "y": -5.7321, "z": -1.5},
                "drone5": {"x": 2.0, "y": -4.0, "z": -1.5},
                "drone6": {"x": 1.0, "y": -2.2679, "z": -1.5},
            },
        }
        self.assertEqual(formation_center(config), (-4.0, 0.0, 1.5))

    def test_rejects_incompatible_shape(self):
        config = {
            "vehicles": [f"drone{i}" for i in range(7)],
            "targets": {vehicle: {"x": 0.0, "y": 0.0, "z": -1.0} for vehicle in [f"drone{i}" for i in range(7)]},
        }
        with self.assertRaises(ValueError):
            formation_center(config)

    def test_rejects_discontinuous_trajectory_start(self):
        with self.assertRaisesRegex(ValueError, "starts 5.000 m"):
            validate_trajectory_starts(
                ["drone0"],
                {"drone0": [0.0, 0.0, 1.0]},
                [[[3.0, 4.0, 1.0], [3.1, 4.0, 1.0]]],
                1.0,
            )

    def test_accepts_continuous_trajectory_start(self):
        validate_trajectory_starts(
            ["drone0"],
            {"drone0": [0.0, 0.0, 1.0]},
            [[[0.1, 0.0, 1.0], [0.2, 0.0, 1.0]]],
            1.0,
        )


if __name__ == "__main__":
    unittest.main()
