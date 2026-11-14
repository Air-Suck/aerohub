"""Captured course registration error agrees with magnetic-model mismatch."""
import unittest
from simulator.projectairsim.magnetic_field import dipole_declination_deg


class MagneticReferenceTest(unittest.TestCase):
    def test_seattle_model_matches_captured_rotation(self):
        declination = dipole_declination_deg(47.641468, -122.140165)
        self.assertAlmostEqual(declination, 7.75167536, places=6)
        self.assertAlmostEqual(15.0112 - declination, 7.36, delta=.15)

    def test_rejects_invalid_geography(self):
        for lat, lon in [(float('nan'), 0), (91, 0), (0, float('inf')), (0, 181)]:
            with self.assertRaises(ValueError):
                dipole_declination_deg(lat, lon)
