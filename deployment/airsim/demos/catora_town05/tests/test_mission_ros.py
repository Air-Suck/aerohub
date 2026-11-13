"""Run in the recorded ROS image. No ROS nodes or motion commands are created."""
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

APP = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('town05_mission', APP / 'mission.py')
mission = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mission)


class RecoveryTest(unittest.TestCase):
    def test_cancel_and_hover_failures_do_not_skip_land_attempts(self):
        client = mission.Town05Mission.__new__(mission.Town05Mission)
        with tempfile.TemporaryDirectory() as directory:
            client.output = Path(directory)
            client.controller = Mock()
            client.vehicles = ['drone0', 'drone1']
            client.modes = {v: SimpleNamespace(flag_armed=True) for v in client.vehicles}
            client.planner_call = Mock(side_effect=RuntimeError('cancel failed'))
            client.controller_mode = Mock(side_effect=RuntimeError('hover failed'))
            def land(vehicle, *_):
                client.modes[vehicle].flag_armed = False
            client.action = Mock(side_effect=land)
            client.wait = lambda predicate, timeout: self.assertTrue(predicate())
            client.recover()
            self.assertEqual(2, client.action.call_count)
            result = json.loads((client.output/'failure-stop.json').read_text())
            self.assertTrue(result['landed_and_disarmed'])
            self.assertEqual(2, len(result['errors']))

    def test_workstation_cannot_launch_compute_processes(self):
        client = mission.Town05Mission.__new__(mission.Town05Mission)
        with self.assertRaisesRegex(RuntimeError, 'must not start'):
            client.start('algorithm', ['not-executed'])
