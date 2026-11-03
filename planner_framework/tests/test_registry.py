import unittest
from copy import deepcopy
from pathlib import Path

import yaml

from uav_planner_framework import ConfigError, PlannerRegistry, recursive_merge


ROOT = Path(__file__).parents[1]
VERSION = "4bc41f593eebde65dd74108a97e80430a145dfa0"


def task():
    return {
        "task": {"name": "test"},
        "planner": {"id": "cat_ora", "version": VERSION, "parameters": {"max_velocity": 2.0}},
        "vehicles": ["uav1"],
        "targets": {"uav1": {"x": 1.0, "y": 2.0, "z": -3.0}},
        "completion": {"position_tolerance": 0.5, "hold_duration": 1.0, "task_timeout": 120.0},
    }


def deployment():
    return {"uav1": {"px4_origin_in_fleet_ned": [0.0, 0.0, 0.0], "namespace": "/uav1", "topics": {}}}


class RegistryTest(unittest.TestCase):
    def setUp(self):
        self.registry = PlannerRegistry(ROOT / "registry")

    def test_recursive_merge_replaces_arrays(self):
        self.assertEqual(recursive_merge({"a": {"b": 1}, "c": [1]}, {"a": {"b": 2}, "c": [3]}),
                         {"a": {"b": 2}, "c": [3]})

    def test_resolve(self):
        resolved = self.registry.resolve(task(), deployment())
        self.assertEqual(resolved["planner"]["parameters"]["max_velocity"], 2.0)
        self.assertEqual(resolved["coordinates"]["frame"], "LOCAL_NED")
        self.assertTrue(resolved["task"]["task_id"])

    def test_parameters_may_be_omitted_to_use_defaults(self):
        value = task()
        del value["planner"]["parameters"]
        resolved = self.registry.resolve(value, deployment())
        self.assertEqual(resolved["planner"]["parameters"]["max_velocity"], 1.2)

    def test_unknown_parameter_is_rejected(self):
        value = task()
        value["planner"]["parameters"]["typo"] = 1
        with self.assertRaisesRegex(ConfigError, "unknown parameter"):
            self.registry.resolve(value, deployment())

    def test_target_set_must_match_vehicles(self):
        value = task()
        value["targets"] = {}
        with self.assertRaisesRegex(ConfigError, "targets must match"):
            self.registry.resolve(value, deployment())

    def test_manifest_resource_types_are_validated(self):
        path = ROOT / "registry" / "cat_ora" / VERSION / "manifest.yaml"
        manifest = yaml.safe_load(path.read_text(encoding="utf-8"))
        invalid = deepcopy(manifest)
        invalid["resources"]["gpu_required"] = "false"
        with self.assertRaisesRegex(ConfigError, "gpu_required"):
            PlannerRegistry._validate_manifest(invalid, "cat_ora", VERSION)

    def test_all_registered_manifests_are_valid(self):
        for path in (ROOT / "registry").glob("*/*/manifest.yaml"):
            manifest = yaml.safe_load(path.read_text(encoding="utf-8"))
            PlannerRegistry._validate_manifest(manifest, path.parents[1].name, path.parent.name)


if __name__ == "__main__":
    unittest.main()
