import json
from pathlib import Path
import tempfile
import unittest

from simulator.manager.config import load_profile, runtime_manifest
from simulator.manager.projectairsim_config import configure_scene


class ConfigTest(unittest.TestCase):
    def test_legacy_profile_defaults_to_airsim(self):
        profile = load_profile(Path("simulator/airsim/config/nh.json"))
        self.assertEqual(profile["backend"], "airsim")

    def test_projectairsim_manifest_preserves_px4_mapping(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ("host", "scene.jsonc"):
                (root / name).write_text("{}")
            (root / "host").chmod(0o755)
            (root / "client").mkdir()
            source = {
                "backend": "projectairsim",
                "binary": "host",
                "scene_config": "scene.jsonc",
                "client_path": "client",
                "client_python": "venv-python",
                "vehicles": [{
                    "name": "Drone1", "namespace": "drone0",
                    "px4": {"tcp_port": 4560, "control_local_port": 14540,
                            "control_remote_port": 14580, "lockstep": True},
                }],
            }
            path = root / "profile.json"
            (root / "venv-python").symlink_to(__import__("sys").executable)
            path.write_text(json.dumps(source))
            profile = load_profile(path)
            self.assertEqual(profile["client_python"], str(root / "venv-python"))
            manifest = runtime_manifest(profile)
            self.assertEqual(manifest["backend"], "projectairsim")
            self.assertEqual(manifest["vehicles"][0]["px4"]["tcp_port"], 4560)

    def test_rejects_duplicate_projectairsim_vehicles(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "host").write_text("")
            (root / "scene").write_text("")
            (root / "client").mkdir()
            vehicle = {"name": "same", "px4": {"tcp_port": 1,
                       "control_local_port": 2, "control_remote_port": 3}}
            path = root / "profile.json"
            path.write_text(json.dumps({"backend": "projectairsim", "binary": "host",
                "scene_config": "scene", "client_path": "client", "vehicles": [vehicle, vehicle]}))
            with self.assertRaisesRegex(ValueError, "duplicate vehicle"):
                load_profile(path)

    def test_resolved_scene_uses_manifest_ports_without_mutating_source(self):
        scene = {"actors": [{"name": "Drone1", "type": "robot", "robot-config": {
            "controller": {"type": "px4-api", "px4-settings": {"tcp-port": 4560}},
            "sensors": [{"id": "camera", "enabled": True}],
        }}]}
        manifest = {"vehicles": [{"name": "Drone1", "px4": {
            "tcp_port": 4572, "control_local_port": 14600,
            "control_remote_port": 14640, "lockstep": True}}]}
        result = configure_scene(scene, manifest)
        robot = result["actors"][0]["robot-config"]
        self.assertEqual(robot["controller"]["px4-settings"]["tcp-port"], 4572)
        self.assertEqual(robot["controller"]["px4-settings"]["control-port-remote"], 14640)
        self.assertEqual(robot["sensors"], scene["actors"][0]["robot-config"]["sensors"])
        self.assertEqual(scene["actors"][0]["robot-config"]["controller"]["px4-settings"],
                         {"tcp-port": 4560})
        manifest["vehicles"][0]["name"] = "missing"
        with self.assertRaisesRegex(ValueError, "match"):
            configure_scene(scene, manifest)

    def test_scene_rejects_non_px4_controller(self):
        scene = {"actors": [{"name": "Drone1", "type": "robot", "robot-config": {
            "controller": {"type": "simple-flight-api"}}}]}
        with self.assertRaisesRegex(ValueError, "px4-api"):
            configure_scene(scene, {"vehicles": [{"name": "Drone1", "px4": {}}]})


if __name__ == "__main__":
    unittest.main()
