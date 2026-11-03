import math
import unittest
from types import SimpleNamespace

from uav_as2_setpoint_adapter.node import ned_to_enu, px4_yaw_to_enu


class ConversionTest(unittest.TestCase):
    def test_ned_to_enu(self):
        position = SimpleNamespace(x=1.0, y=2.0, z=-3.0)
        self.assertEqual(ned_to_enu(position), (2.0, 1.0, 3.0))

    def test_px4_yaw_to_enu(self):
        self.assertAlmostEqual(px4_yaw_to_enu(0.0), math.pi / 2.0)
        self.assertAlmostEqual(px4_yaw_to_enu(math.pi / 2.0), 0.0)


if __name__ == "__main__":
    unittest.main()
