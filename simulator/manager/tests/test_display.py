"""Configuration and explicit event transport for the optional UE display."""
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from simulator.manager.display import request_display, write_display_config
from simulator.manager.yaml_config import projectairsim_profile


class DisplayTest(unittest.TestCase):
    def config(self):
        return {"projectairsim": {"source": "/sim", "binary": "/sim/Blocks",
                 "python": "/sim/.venv/bin/python", "ros_setup": "/sim/setup.bash"},
                "vehicles": [{"name": "Drone1"}], "display": {"camera": "FrontCamera"}}

    def test_display_defaults_and_invalid_port(self):
        config = self.config()
        profile = projectairsim_profile(config, Path('/config/default.yaml'))
        self.assertEqual(profile['display']['camera'], 'FrontCamera')
        self.assertEqual(profile['display']['vehicle'], 'Drone1')
        self.assertFalse(profile['display']['camera_enabled'])
        self.assertEqual(profile['display']['speed_max_mps'], 20)
        config['display']['port'] = 8990
        with self.assertRaisesRegex(ValueError, 'distinct valid port'):
            projectairsim_profile(config, Path('/config/default.yaml'))

    def test_attach_config_does_not_modify_map(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            project = root / 'Blocks.uproject'
            project.write_text('{}')
            profile = {'editor_project': str(project), 'display': {'camera': 'FrontCamera'}}
            target = write_display_config(profile, root / 'runtime', attach=True)
            self.assertEqual(target, root / 'Saved/Aerohub/display.json')
            self.assertEqual(json.loads(target.read_text()), profile['display'])
            self.assertEqual(project.read_text(), '{}')
            self.assertEqual(write_display_config(profile, root / 'runtime'), root / 'runtime/display.json')

    def test_timer_event_is_explicit_post(self):
        with patch('simulator.manager.display.urlopen', return_value=io.BytesIO(b'{"mission":"running"}')) as call:
            self.assertEqual(request_display({'display': {'port': 8992}}, 'start')['mission'], 'running')
            request = call.call_args.args[0]
            self.assertEqual(request.full_url, 'http://127.0.0.1:8992/aerohub/mission/start')
            self.assertEqual(request.method, 'POST')
            self.assertEqual(request.data, b'')
