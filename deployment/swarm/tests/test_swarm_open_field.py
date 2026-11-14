"""Swarm-Formation hybrid entry: generated artifacts and PC worker lifecycle, without Docker/RoboPi/ROS."""
import ast
import copy
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

import yaml

from deployment.swarm.topology import ConfigError, load_config, resolve_topology


ROOT = Path(__file__).resolve().parents[3]
APP = ROOT / "deployment/airsim/demos/swarm_open_field"
CONFIG = ROOT / "deployment/swarm/configs/swarm_open_field.yaml"
ADAPTER = ROOT / "src/tool/navigation/planner/swarm_formation/ros2_adapter/bridge/uav_swarm_formation_adapter/node.py"
FAKE_DOCKER = ROOT / "deployment/swarm/tests/fake_docker.py"
sys.path.insert(0, str(APP))

import hybrid_plan  # noqa: E402
import run_fault_matrix  # noqa: E402
from orchestrate import Workers  # noqa: E402


TASK = yaml.safe_load((APP / "task.yaml").read_text())
PLANNER = yaml.safe_load((APP / "planner.yaml").read_text())
NATIVE = yaml.safe_load(hybrid_plan.NATIVE_PARAMETERS.read_text())


def configured(count, board_index):
    config = load_config(CONFIG)
    config["fleet"]["vehicle_count"] = count
    config["fleet"]["board_vehicle_index"] = board_index
    return config


def plan(count, board_index, scenario):
    return hybrid_plan.build_plan(configured(count, board_index), scenario, TASK, PLANNER, NATIVE)


def adapter_relative_enu():
    tree = ast.parse(ADAPTER.read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == "RELATIVE_ENU" for target in node.targets
        ):
            return ast.literal_eval(node.value)
    raise AssertionError("RELATIVE_ENU not found in the Swarm adapter")


class HybridPlanTest(unittest.TestCase):
    CASES = [(3, 0), (3, 2), (5, 0), (5, 3), (7, 6)]

    def test_count_and_board_index_flow_into_every_artifact(self):
        for count, board_index in self.CASES:
            for scenario in ("board_center", "board_worker"):
                with self.subTest(count=count, board=board_index, scenario=scenario):
                    result = plan(count, board_index, scenario)
                    names = [f"drone{index}" for index in range(count)]
                    airsim = [f"Drone{index}" for index in range(count)]
                    vehicles = result["settings"]["Vehicles"]
                    self.assertEqual(airsim, list(vehicles))
                    self.assertEqual(names, result["task"]["vehicles"])
                    self.assertEqual(names, list(result["task"]["targets"]))
                    self.assertEqual(names, list(result["deployment"]))
                    self.assertEqual(names, list(result["fleet"]["vehicles"]))
                    self.assertEqual(airsim, [item["airsim_name"] for item in result["sim_vehicles"]])
                    self.assertEqual(f"swarm_nh_open_field_{count}", result["task"]["task"]["name"])
                    for index, vehicle in enumerate(result["topology"]["vehicles"]):
                        rendered = vehicles[airsim[index]]
                        self.assertEqual(4680 + index, rendered["TcpPort"])
                        self.assertEqual(14840 + index, rendered["ControlPortLocal"])
                        self.assertEqual(14980 + index, rendered["ControlPortRemote"])
                        self.assertEqual(
                            {"name": names[index], "airsim_name": airsim[index], "px4_instance": 40 + index,
                             "px4_system_id": 41 + index, "xrce_agent_port": 8901 + index},
                            result["sim_vehicles"][index],
                        )
                        self.assertEqual(41 + index, result["fleet"]["vehicles"][names[index]]["target_system_id"])
                        origin = result["deployment"][names[index]]["px4_origin_in_fleet_ned"]
                        self.assertEqual([rendered["X"], rendered["Y"], 0.0], origin)
                        self.assertEqual(vehicle["initial_xy_m"], [rendered["X"], rendered["Y"]])
                    board = f"drone{board_index}"
                    self.assertEqual(board, result["topology"]["board_vehicle"]["name"])
                    self.assertEqual([board], list(result["board_fleet"]["vehicles"]))
                    pc_indices = [index for index in range(count) if index != board_index]
                    self.assertEqual(
                        [hybrid_plan.PREFIX + f"compute{index}" for index in pc_indices],
                        result["containers"]["compute"],
                    )
                    self.assertEqual(count, len(result["containers"]["vehicles"]))
                    self.assertEqual(count, len(result["containers"]["sensors"]))
                    all_names = [name for group in result["containers"].values() for name in group]
                    self.assertEqual(len(all_names), len(set(all_names)))

    def test_center_role_is_separate_from_placement(self):
        for count, board_index in self.CASES:
            center = plan(count, board_index, "board_center")
            worker = plan(count, board_index, "board_worker")
            board = f"drone{board_index}"
            self.assertEqual(board, center["scenario"]["center_vehicle"])
            self.assertEqual("robopi", center["roles"]["center_coordinator"]["placement"])
            self.assertNotEqual(board, worker["scenario"]["center_vehicle"])
            self.assertIn(
                worker["scenario"]["center_vehicle"],
                [vehicle["name"] for vehicle in worker["topology"]["pc_vehicles"]],
            )
            self.assertEqual("pc", worker["roles"]["center_coordinator"]["placement"])
            for result in (center, worker):
                roles = result["roles"]
                self.assertEqual(board, roles["board_uav_node"]["vehicle"])
                self.assertEqual("robopi", roles["board_uav_node"]["placement"])
                self.assertEqual(count - 1, len(roles["pc_uav_nodes"]))
                self.assertNotIn(board, [node["vehicle"] for node in roles["pc_uav_nodes"]])
                self.assertEqual("pc", roles["task_client"]["placement"])
                self.assertTrue(roles["ground_station_on_pc"])
            self.assertEqual([], center["roles"]["known_gaps"])
            self.assertEqual([], worker["roles"]["known_gaps"])

    def test_unsupported_fleet_sizes_are_rejected_by_the_demo_not_the_topology(self):
        for count in (2, 8):
            config = configured(count, 0)
            self.assertEqual(count, resolve_topology(config)["vehicle_count"])
            with self.assertRaisesRegex(ConfigError, "supports 3..7"):
                hybrid_plan.build_plan(config, "board_center", TASK, PLANNER, NATIVE)

    def test_vehicle_names_must_match_native_ids(self):
        config = configured(3, 0)
        config["fleet"]["name_prefix"] = "uav"
        with self.assertRaisesRegex(ConfigError, "drone<i>"):
            hybrid_plan.build_plan(config, "board_center", TASK, PLANNER, NATIVE)

    def test_task_and_native_formation_scale_must_agree(self):
        task = copy.deepcopy(TASK)
        task["planner"]["parameters"]["formation_scale"] = 1.0
        with self.assertRaisesRegex(ConfigError, "formation_scale"):
            hybrid_plan.build_plan(configured(3, 0), "board_center", task, PLANNER, NATIVE)

    def test_targets_reproduce_the_verified_three_vehicle_task(self):
        vehicles = [
            {"name": name, "initial_xy_m": origin[:2]}
            for name, origin in (
                (name, item["px4_origin_in_fleet_ned"])
                for name, item in yaml.safe_load((APP / "deployment.yaml").read_text()).items()
            )
        ]
        _, offsets = hybrid_plan.formation_offsets_enu(3, PLANNER, NATIVE)
        targets = hybrid_plan.formation_targets(vehicles, offsets)
        for name, expected in TASK["targets"].items():
            for axis in "xyz":
                self.assertAlmostEqual(float(expected[axis]), targets[name][axis], places=4)

    def test_targets_satisfy_adapter_formation_check_in_enu(self):
        relative = adapter_relative_enu()
        for count in range(3, 8):
            for board_index in (0, count - 1):
                result = plan(count, board_index, "board_worker")
                scale = float(result["task"]["planner"]["parameters"]["formation_scale"])
                centers = []
                for name in result["task"]["vehicles"]:
                    target = result["task"]["targets"][name]
                    enu = (target["y"], target["x"], -target["z"])
                    centers.append(tuple(enu[axis] - scale * relative[name][axis] for axis in range(3)))
                for center in centers:
                    self.assertLess(math.dist(center, centers[0]), 0.05)

    def test_targets_keep_north_east_order_for_asymmetric_layout(self):
        vehicles = [
            {"name": "drone0", "initial_xy_m": [10.0, -3.0]},
            {"name": "drone1", "initial_xy_m": [12.0, -1.0]},
            {"name": "drone2", "initial_xy_m": [11.0, 1.0]},
        ]
        offsets = [(0.0, 0.0, 0.0), (3.0, -1.0, 0.0), (0.0, -2.0, 0.0)]
        targets = hybrid_plan.formation_targets(vehicles, offsets)
        north = sum(target["x"] for target in targets.values()) / 3
        east = sum(target["y"] for target in targets.values()) / 3
        self.assertAlmostEqual(11.0 + 6.0, north)
        self.assertAlmostEqual(-1.0, east)
        # ENU offset (east=3, north=-1) must become NED (north -1, east +3).
        self.assertAlmostEqual(-1.0, targets["drone1"]["x"] - targets["drone0"]["x"])
        self.assertAlmostEqual(3.0, targets["drone1"]["y"] - targets["drone0"]["y"])
        self.assertTrue(all(target["z"] == -3.0 for target in targets.values()))

    def test_written_artifacts_round_trip_in_index_order(self):
        result = plan(7, 4, "board_worker")
        base = json.loads((ROOT / "simulator/airsim/scenarios/swarm_open_field/profile.json").read_text())
        with tempfile.TemporaryDirectory() as temporary:
            out = Path(temporary)
            camera = {"name": "swarm_overhead", "altitude_m": 25}
            paths = hybrid_plan.write_plan(result, out, base, camera)
            self.assertEqual(result["settings"], json.loads(paths["settings"].read_text()))
            profile = json.loads(paths["profile"].read_text())
            self.assertEqual(str(paths["settings"]), profile["settings"])
            self.assertEqual(camera, profile["overhead_view"])
            self.assertEqual(base["binary_resource"], profile["binary_resource"])
            for key in ("task", "deployment", "fleet", "board_fleet"):
                self.assertEqual(result[key], yaml.safe_load(paths[key].read_text()))
            self.assertEqual(
                [f"drone{index}" for index in range(7)],
                list(yaml.safe_load(paths["deployment"].read_text())),
            )


class FaultMatrixEvidenceTest(unittest.TestCase):
    def test_requires_result_and_clean_runtime_shutdown(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            (output / "scene").mkdir()
            (output / "result.json").write_text(json.dumps({
                "planner_status": "FAILED",
                "as2_hover_confirmed": True,
                "landed_and_disarmed": True,
            }))
            (output / "pc-worker-final.json").write_text(json.dumps([{
                "error": None, "thread_alive": False, "container_present": False,
            }]))
            (output / "board-cleanup.json").write_text(json.dumps({"remaining": {}}))
            (output / "scene/state.json").write_text(json.dumps({"phase": "stopped"}))
            self.assertTrue(run_fault_matrix.validate_evidence(output))
            (output / "cleanup-errors.json").write_text("[]")
            self.assertFalse(run_fault_matrix.validate_evidence(output))
            (output / "cleanup-errors.json").unlink()
            (output / "pc-worker-final.json").write_text(json.dumps([{
                "error": None, "thread_alive": True, "container_present": False,
            }]))
            self.assertFalse(run_fault_matrix.validate_evidence(output))


class FakeDockerCase(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.state = root / "docker-state"
        self.state.mkdir()
        shim = root / "bin"
        shim.mkdir()
        (shim / "docker").write_text(f'#!/bin/sh\nexec "{sys.executable}" "{FAKE_DOCKER}" "$@"\n')
        (shim / "docker").chmod(0o755)
        self.environment = {key: os.environ.get(key) for key in ("PATH", "FAKE_DOCKER_STATE")}
        os.environ["PATH"] = f"{shim}{os.pathsep}{os.environ['PATH']}"
        os.environ["FAKE_DOCKER_STATE"] = str(self.state)
        self.out = root / "run"
        self.out.mkdir()
        (self.out / "pc-dds.xml").write_text("<dds/>\n")

    def tearDown(self):
        for key, value in self.environment.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
        self.temporary.cleanup()

    def behave(self, **behavior):
        (self.state / "behavior.json").write_text(json.dumps(behavior))

    def events(self):
        path = self.state / "events.jsonl"
        return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []

    def workers(self, count, board_index, scenario="board_worker", **options):
        result = plan(count, board_index, scenario)
        return result, Workers(
            self.out, APP, hybrid_plan.PREFIX, "fake/compute:test", "/remote/run",
            lambda command, timeout=180: "[]", lambda *arguments, **kwargs: None,
            result["topology"]["pc_vehicles"], result["fleet"], count,
            result["scenario"]["name"], result["scenario"]["center_vehicle"], **options,
        )

    def assert_clean(self, workers):
        self.assertEqual([], list(self.state.glob("*.pid")))
        for worker in workers.workers:
            self.assertFalse(worker.is_alive())
        self.assertFalse(any(record["thread_alive"] or record["container_present"] for record in workers.final))
        self.assertEqual(workers.final, json.loads((self.out / "pc-worker-final.json").read_text()))


class ComputeWorkerLifecycleTest(FakeDockerCase):
    def test_each_pc_uav_has_one_thread_and_one_process_and_cleanup_leaves_nothing(self):
        result, workers = self.workers(5, 2)
        before = {thread.ident for thread in threading.enumerate()}
        try:
            workers.start()
            records = json.loads((self.out / "pc-worker-topology.json").read_text())
            self.assertEqual(result["containers"]["compute"], [record["container"] for record in records])
            self.assertEqual(4, len({record["thread_id"] for record in records}))
            self.assertEqual(4, len({record["docker_pid"] for record in records}))
            self.assertTrue(all(record["thread_id"] not in before for record in records))
            self.assertNotIn("drone2", [record["vehicle"] for record in records])
            runs = [event for event in self.events() if event["event"] == "run"]
            self.assertEqual(sorted(result["containers"]["compute"]), sorted(event["name"] for event in runs))
            self.assertEqual(
                {record["container"]: record["docker_pid"] for record in records},
                {event["name"]: event["pid"] for event in runs},
            )
            self.assertEqual(4, len(list(self.state.glob("*.pid"))))
        finally:
            final = workers.stop()
        self.assertEqual([], workers.failures())
        self.assertTrue(all(record["returncode"] == 0 and record["error"] is None for record in final))
        self.assert_clean(workers)
        for vehicle in result["topology"]["pc_vehicles"]:
            root = self.out / "compute" / vehicle["name"]
            log = (root / "container.log").read_text()
            container = hybrid_plan.PREFIX + f"compute{vehicle['index']}"
            self.assertIn(f"started {container}", log)
            self.assertIn(f"stopped {container} code=0", log)
            self.assertEqual({vehicle["name"]}, set(yaml.safe_load((root / "fleet.yaml").read_text())["vehicles"]))

    def test_exit_during_startup_fails_and_other_workers_are_cleaned(self):
        _, workers = self.workers(3, 0)
        failing = hybrid_plan.PREFIX + "compute1"
        self.behave(exit={failing: [0.0, 3]})
        try:
            with self.assertRaisesRegex(RuntimeError, "failed during startup"):
                workers.start()
        finally:
            workers.stop()
        failures = workers.failures()
        self.assertEqual([failing], [record["container"] for record in failures])
        self.assertIn("code 3", failures[0]["error"])
        self.assert_clean(workers)

    def test_exit_after_startup_is_a_failure_even_with_code_zero(self):
        _, workers = self.workers(3, 1)
        failing = hybrid_plan.PREFIX + "compute2"
        self.behave(exit={failing: [1.5, 0]})
        try:
            workers.start()
            worker = next(item for item in workers.workers if item.container == failing)
            worker.join(timeout=10)
            self.assertFalse(worker.is_alive())
        finally:
            workers.stop()
        failures = workers.failures()
        self.assertEqual([failing], [record["container"] for record in failures])
        self.assertIn("exited before stop with code 0", failures[0]["error"])
        self.assert_clean(workers)

    def test_unstoppable_container_is_force_removed(self):
        _, workers = self.workers(3, 0, stop_timeout=0.2)
        stuck = hybrid_plan.PREFIX + "compute2"
        self.behave(ignore_stop=[stuck])
        started = time.monotonic()
        try:
            workers.start()
        finally:
            workers.stop()
        self.assertLess(time.monotonic() - started, 30)
        self.assertIn({"event": "rm", "name": stuck}, self.events())
        self.assertEqual([], workers.failures())
        self.assert_clean(workers)

    def test_stop_before_start_is_safe(self):
        _, workers = self.workers(3, 0)
        workers.stop()
        self.assertEqual([], workers.failures())
        self.assertTrue(all(record["thread_id"] is None for record in workers.final))
        self.assert_clean(workers)


if __name__ == "__main__":
    unittest.main()
