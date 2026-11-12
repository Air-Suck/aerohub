"""Host ownership and cleanup boundaries for editor sessions."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from simulator.manager.manager import _alive, _terminate, _start_projectairsim


class EditorSessionTest(unittest.TestCase):
    def test_stop_never_signals_external_host(self):
        state = {"owns_host": False, "phase": "ready", "pid": 123,
                 "auxiliaries": [{"pid": 456, "start_ticks": "1"}]}
        with tempfile.TemporaryDirectory() as directory, patch(
            "simulator.manager.manager._process_alive", side_effect=[True, False, False]
        ), patch("simulator.manager.manager._alive", return_value=True), patch(
            "simulator.manager.manager.os.killpg"
        ) as kill:
            _terminate(state, Path(directory))
            self.assertEqual(kill.call_count, 1)
            self.assertEqual(kill.call_args.args[0], 456)
            self.assertEqual(state["phase"], "stopped")

    def test_attach_failure_does_not_launch_or_kill_host(self):
        profile = {"backend": "projectairsim", "vehicles": [], "client_python": "python",
                   "client_path": "/client", "scene_config": "/scene",
                   "ports": {"topics": 8989, "services": 8990}}
        with tempfile.TemporaryDirectory() as directory, patch(
            "simulator.manager.manager.subprocess.run", side_effect=RuntimeError("load failed")
        ), patch("simulator.manager.manager.subprocess.Popen") as launch, patch(
            "simulator.manager.manager.os.killpg"
        ) as kill:
            with self.assertRaisesRegex(RuntimeError, "load failed"):
                _start_projectairsim(profile, Path(directory), 1, attach=True)
            launch.assert_not_called()
            kill.assert_not_called()
            state = json.loads((Path(directory) / "state.json").read_text())
            self.assertFalse(state["owns_host"])
            self.assertEqual(state["phase"], "stopped")

    def test_editor_stopped_is_unhealthy(self):
        state = {"owns_host": False, "phase": "ready", "profile": {
            "ports": {"topics": 8989, "services": 8990}}}
        with patch("simulator.manager.manager.socket.create_connection", side_effect=OSError):
            self.assertFalse(_alive(state))

    def test_attach_owns_display_telemetry_as_auxiliary(self):
        profile = {"backend": "projectairsim", "vehicles": [], "client_python": "python",
                   "client_path": "/client", "scene_config": "/scene",
                   "ports": {"topics": 8989, "services": 8990},
                   "display": {"enabled": True, "vehicle": "Drone1", "port": 8991}}
        with tempfile.TemporaryDirectory() as directory, patch(
            "simulator.manager.manager.subprocess.run"
        ) as load, patch("simulator.manager.manager.subprocess.Popen") as launch, patch(
            "simulator.manager.manager._ticks", return_value="1"
        ):
            profile["editor_project"] = str(Path(directory) / "Blocks.uproject")
            load.return_value.returncode = 0
            launch.return_value.pid = 456
            state = _start_projectairsim(profile, Path(directory), 1, attach=True)
            self.assertFalse(state["owns_host"])
            self.assertEqual(state["auxiliaries"][0]["role"], "display-telemetry")
            self.assertIn("--display-port", launch.call_args.args[0])
