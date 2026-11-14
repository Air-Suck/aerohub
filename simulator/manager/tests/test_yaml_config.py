from copy import deepcopy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml

from simulator.manager.config import load_profile
from simulator.projectairsim.scene_config import configure_sensors


class YamlConfigTest(unittest.TestCase):
    def test_simple_yaml_compiles_paths_ports_and_bridge(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ("Blocks", "python", "setup.bash"):
                (root / name).touch()
            (root / "client/python/projectairsim/src").mkdir(parents=True)
            (root / "client/python/example_user_scripts/sim_config").mkdir(parents=True)
            (root / "client/python/example_user_scripts/sim_config/scene_px4_sitl.jsonc").touch()
            config = {"backend": "projectairsim", "render": "offscreen", "ros_domain_id": 17,
                "projectairsim": {"source": ".", "binary": "Blocks", "python": "python", "ros_setup": "setup.bash"},
                "vehicles": [{"name": "Drone1", "namespace": "drone0",
                    "sensors": {"cam": {"width": 640}}}]}
            path = root / "simulator.yaml"
            path.write_text(yaml.safe_dump(config))
            profile = load_profile(path)
            self.assertEqual(profile["binary"], str(root / "Blocks"))
            self.assertEqual(profile["vehicles"][0]["px4"]["tcp_port"], 4560)
            self.assertEqual(profile["adapter"]["setup"], str(root / "setup.bash"))
            self.assertEqual(profile["adapter"]["environment"]["ROS_DOMAIN_ID"], "17")
            self.assertEqual(profile["sensor_options"], {"Drone1": {"cam": {"width": 640}}})
            self.assertIn("-RenderOffscreen", profile["args"])
            self.assertNotIn("sensors", profile["vehicles"][0])

    def test_resource_paths_and_local_override_preserve_venv(self):
        from simulator.manager.resources import resolve_resource
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "config").mkdir()
            (root / "config/resources.yaml").write_text(
                "resources:\n  python:\n    path: ../shared/python\n")
            (root / "config/resources.local.yaml").write_text(
                "resources:\n  python:\n    path: ../local/bin/python\n")
            target = root / "local/bin"
            target.mkdir(parents=True)
            (target / "python").symlink_to("/usr/bin/python3")
            with patch("simulator.manager.resources.__file__",
                       str(root / "simulator/manager/resources.py")), patch.dict(
                           "os.environ", {"XDG_CONFIG_HOME": str(root / "xdg")}):
                result = resolve_resource("python")
            self.assertEqual(result, root / "config/../local/bin/python")
            self.assertNotEqual(str(result), "/usr/bin/python3")

    def test_sensor_settings_preserve_native_model_and_source(self):
        camera = {"id": "cam", "type": "camera", "parent-link": "Frame",
                  "capture-settings": [{"image-type": 0, "width": 400, "height": 225,
                      "fov-degrees": 90, "capture-enabled": True}]}
        scene = {"actors": [{"type": "robot", "name": "Drone1",
                  "robot-config": {"sensors": [camera], "links": [{"name": "Frame"}]}}]}
        original = deepcopy(scene)
        lidar = {"id": "official_lidar", "type": "lidar", "parent-link": "Frame",
                 "range": 100, "vertical-fov-lower-deg": -90}
        options = {"Drone1": {"cam": {"width": 640, "height": 480, "fov": 80,
            "position": [1, 2, 3], "captures": ["rgb", "depth_planar"]},
            "lidar1": {"type": "lidar", "channels": 32}}}
        result = configure_sensors(scene, options, lidar)
        sensors = result["actors"][0]["robot-config"]["sensors"]
        self.assertEqual(sensors[0]["origin"]["xyz"], "1 2 3")
        self.assertEqual({c["image-type"] for c in sensors[0]["capture-settings"]}, {0, 1})
        self.assertTrue(all(c["width"] == 640 for c in sensors[0]["capture-settings"]))
        self.assertEqual(sensors[1]["number-of-channels"], 32)
        self.assertEqual(sensors[1]["vertical-fov-lower-deg"], -90)
        self.assertEqual(scene, original)
        with self.assertRaisesRegex(ValueError, "unknown sensor"):
            configure_sensors(scene, {"Drone1": {"cam": {"widht": 10}}})


if __name__ == "__main__":
    unittest.main()
