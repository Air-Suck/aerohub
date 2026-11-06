"""Prevent silent reuse of a base image's unrelated demonstration target."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

source = Path(__file__).resolve().parents[3] / "deployment/swarm_formation_ros1/configure_waypoints.py"
spec = importlib.util.spec_from_file_location("waypoints", source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class WaypointConfigTest(unittest.TestCase):
    def test_replaces_arbitrary_defaults_and_is_idempotent(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "launch.xml"
            root = ET.Element("launch")
            include = ET.SubElement(root, "include", file="$(find ego_planner)/launch/advanced_param.xml")
            for name, value in {"point_num": "4", "point0_x": "24.0",
                                "point0_y": "3.5", "point0_z": "1.2"}.items():
                ET.SubElement(include, "arg", name=name, value=value)
            ET.ElementTree(root).write(path)
            module.configure(path)
            module.configure(path)
            values = {arg.get("name"): arg.get("value")
                      for arg in ET.parse(path).getroot().find("include")}
            self.assertEqual(values, {"point_num": "1", "point0_x": "$(arg target_x)",
                                      "point0_y": "$(arg target_y)", "point0_z": "$(arg target_z)"})

    def test_missing_argument_fails_without_writing(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "launch.xml"
            ET.ElementTree(ET.Element("launch")).write(path)
            before = path.read_bytes()
            with self.assertRaises(ValueError):
                module.configure(path)
            self.assertEqual(before, path.read_bytes())


if __name__ == "__main__":
    unittest.main()
