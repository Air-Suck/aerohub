import unittest

from uav_planner_framework import BehaviorLifecycle, BehaviorStatus, TransitionError


class BehaviorLifecycleTest(unittest.TestCase):
    def test_pause_resume_cancel(self):
        lifecycle = BehaviorLifecycle()
        self.assertEqual(lifecycle.start("task-1").status, BehaviorStatus.RUNNING)
        self.assertEqual(lifecycle.pause().status, BehaviorStatus.PAUSED)
        self.assertEqual(lifecycle.resume().status, BehaviorStatus.RUNNING)
        self.assertEqual(lifecycle.cancel().status, BehaviorStatus.CANCELED)
        with self.assertRaises(TransitionError):
            lifecycle.resume()

    def test_failure_requires_reason(self):
        lifecycle = BehaviorLifecycle()
        lifecycle.start("task-1")
        with self.assertRaises(TransitionError):
            lifecycle.fail("NONE")
        snapshot = lifecycle.fail("NO_SOLUTION")
        self.assertEqual(snapshot.status, BehaviorStatus.FAILED)
        self.assertEqual(snapshot.failure_reason, "NO_SOLUTION")

    def test_second_start_preserves_active_behavior(self):
        lifecycle = BehaviorLifecycle()
        active = lifecycle.start("task-1")
        with self.assertRaises(TransitionError):
            lifecycle.start("task-2")
        self.assertEqual(lifecycle.snapshot, active)


if __name__ == "__main__":
    unittest.main()
