import importlib.util
from pathlib import Path
import unittest
import numpy as np
from uav_yopo_planner.core import Curve, Timeline, calculate_yaw, wrap

ROOT = Path(__file__).resolve().parents[5]
spec = importlib.util.spec_from_file_location('original_poly', ROOT/'src/planner/yopo/upstream/YOPO/policy/poly_solver.py')
original = importlib.util.module_from_spec(spec)
spec.loader.exec_module(original)


class PlanningTest(unittest.TestCase):
    def setUp(self):
        self.initial = np.array([[1., 2., 3.], [.3, -.8, .4], [.1, .2, -.1]])
        self.terminal = np.array([[5., -1., 4.], [-.1, .5, .3], [-.2, .1, .4]])
        self.curve = Curve.create(10., self.initial, self.terminal, 1.7, 3.13, np.array([-8., -.1, 3.]))

    def test_original_poly_full_domain(self):
        for axis in range(3):
            solver = original.Poly5Solver(*self.initial[:, axis], *self.terminal[:, axis], 1.7)
            for t in np.linspace(0, 1.7, 201):
                pva, _ = self.curve.sample(10.+t)
                expected = [solver.get_position(t), solver.get_velocity(t), solver.get_acceleration(t)]
                np.testing.assert_allclose(pva[:, axis], expected, atol=1e-10)

    def test_duration_rounding_keeps_ros_times_strictly_increasing(self):
        curve = Curve.create(10., self.initial, self.terminal, 5.000000000000001,
                             0., np.array([8., 0., 3.]))
        stamps = [round(t*1e9) for t in curve.times]
        self.assertTrue(all(b > a for a, b in zip(stamps, stamps[1:])))
        self.assertEqual(stamps[-1], 5000000000)
        self.assertEqual(len(stamps), 251)

    def test_yaw_wrap_and_original_policy(self):
        last = 3.13
        for _ in range(100):
            velocity, goal = np.array([-1., -.1, .4]), np.array([-10., -.1, 2.])
            actual = calculate_yaw(velocity, goal, last, .02)
            expected, _ = original.calculate_yaw(velocity, goal, last, .02)
            self.assertAlmostEqual(actual, expected)
            self.assertLessEqual(abs(wrap(actual-last)), .5*np.pi*.02+1e-12)
            last = actual

    def test_pending_rejection_and_future_handoff(self):
        timeline = Timeline()
        self.assertEqual(timeline.propose(self.curve), 1)
        self.assertIsNone(timeline.latest)
        timeline.acknowledge(True)
        pva, yaw = timeline.reference(10.4)
        new = Curve.create(10.4, pva, self.terminal, 1.7, yaw, np.array([8., 2., 3.]))
        timeline.propose(new)
        self.assertIs(timeline.latest, self.curve)
        with self.assertRaises(ValueError):
            timeline.propose(new)
        timeline.acknowledge(False)
        self.assertIs(timeline.latest, self.curve)
        timeline.propose(new)
        timeline.acknowledge(True)
        np.testing.assert_allclose(timeline.reference(10.40001)[0], new.sample(10.40001)[0])
        self.assertFalse(timeline.feedback(999, .2, 10.6, False))
        self.assertTrue(timeline.feedback(2, .2, 10.6, False))
        self.assertEqual(timeline.effective, 2)
        self.assertFalse(timeline.feedback(1, .6, 10.6, False))
        with self.assertRaises(ValueError):
            timeline.reference(12.2)

    def test_feedback_does_not_release_first_mode_switch_early(self):
        timeline = Timeline()
        timeline.propose(self.curve); timeline.acknowledge(True)
        timeline.feedback(1, 0., 10.14, False)
        self.assertEqual(timeline.effective, 0)
        timeline.feedback(1, .14, 10.14, False)
        self.assertEqual(timeline.effective, 1)


if __name__ == '__main__':
    unittest.main()
