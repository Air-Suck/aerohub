import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from deployment.swarm.overhead_view import centroid, pose_position, update_camera
from deployment.swarm.orchestrator import run_matrix
from deployment.swarm.topology import (
    ConfigError,
    load_config,
    render_airsim_settings,
    resolve_topology,
    validate_config,
)
from deployment.swarm.view import vehicle_centroid


ROOT = Path(__file__).resolve().parents[3]
EXAMPLE = ROOT / "deployment/swarm/config.example.yaml"
FAKE = ROOT / "deployment/swarm/tests/fake_service.py"


class SwarmTopologyTest(unittest.TestCase):
    def setUp(self):
        self.config = load_config(EXAMPLE)

    def test_pc_simulation_uses_one_onboard_container_per_vehicle(self):
        for count in (1, 3, 6):
            config = copy.deepcopy(self.config)
            config["fleet"]["vehicle_count"] = count
            topology = resolve_topology(config)
            self.assertEqual("pc_simulation", topology["deployment_stage"])
            self.assertIsNone(topology["board_vehicle"])
            self.assertEqual(
                count, len(topology["onboard_computer_vehicles"])
            )
            self.assertTrue(
                all(
                    vehicle["supervisor"] == "independent_thread"
                    for vehicle in topology["onboard_computer_vehicles"]
                )
            )

    def hybrid_config(self, count=3, board_index=0):
        config = copy.deepcopy(self.config)
        config["deployment"]["stage"] = "hybrid_board"
        config["fleet"]["vehicle_count"] = count
        config["fleet"]["board_vehicle_index"] = board_index
        config["coordination"]["centralized_cases"] = ["board_center", "board_worker"]
        return config

    def test_hybrid_board_uses_one_board_and_n_minus_one_containers(self):
        for count in (2, 3, 6):
            topology = resolve_topology(self.hybrid_config(count, count - 1))
            self.assertEqual("hybrid_board", topology["deployment_stage"])
            self.assertEqual(
                1,
                sum(
                    vehicle["placement"] == "physical_board"
                    for vehicle in topology["vehicles"]
                ),
            )
            self.assertEqual(count - 1, len(topology["onboard_computer_vehicles"]))

    def test_hybrid_centralized_matrix_covers_both_board_roles(self):
        topology = resolve_topology(self.hybrid_config())
        self.assertEqual(
            ["board_center", "board_worker"],
            [scenario["name"] for scenario in topology["scenarios"]],
        )
        board = topology["board_vehicle"]["name"]
        board_worker = topology["scenarios"][1]
        self.assertNotEqual(board, board_worker["center_vehicle"])
        self.assertIn(
            board_worker["center_vehicle"],
            [vehicle["name"] for vehicle in topology["onboard_computer_vehicles"]],
        )

    def test_board_index_is_ignored_in_pc_simulation(self):
        config = copy.deepcopy(self.config)
        config["fleet"]["board_vehicle_index"] = 99
        topology = resolve_topology(config)
        self.assertIsNone(topology["board_vehicle"])

    def test_hybrid_requires_a_valid_board_index(self):
        config = self.hybrid_config()
        config["fleet"].pop("board_vehicle_index")
        with self.assertRaisesRegex(ConfigError, "board_vehicle_index"):
            validate_config(config)

    def test_invalid_ground_station_placement_is_rejected(self):
        config = copy.deepcopy(self.config)
        config["ground_station"]["placement"] = "board"
        with self.assertRaisesRegex(ConfigError, "placement must be pc"):
            validate_config(config)

    def test_onboard_computer_image_is_required(self):
        config = copy.deepcopy(self.config)
        config["nodes"]["onboard_computer"]["image"] = ""
        with self.assertRaisesRegex(ConfigError, "onboard_computer.image"):
            validate_config(config)

    def test_generated_camera_is_top_down_at_single_and_multi_centroid(self):
        for count in (1, 4):
            config = copy.deepcopy(self.config)
            config["fleet"]["vehicle_count"] = count
            if count == 1:
                config["coordination"]["mode"] = "decentralized"
                config["coordination"]["centralized_cases"] = []
            settings = render_airsim_settings(config)
            centroid = vehicle_centroid(settings)
            camera = settings["ExternalCameras"]["swarm_overhead"]
            self.assertAlmostEqual(centroid[0], camera["X"])
            self.assertAlmostEqual(centroid[1], camera["Y"])
            self.assertAlmostEqual(centroid[2] - 25.0, camera["Z"])
            self.assertEqual(-90.0, camera["Pitch"])
            self.assertEqual("Manual", settings["ViewMode"])
            self.assertTrue(settings["SubWindows"][0]["Visible"])


class OverheadTrackerTest(unittest.TestCase):
    def test_centroid_uses_all_live_vehicle_poses(self):
        points = [(1.0, 3.0, -2.0), (5.0, -1.0, -4.0), (0.0, 4.0, 0.0)]
        self.assertEqual((2.0, 2.0, -2.0), centroid(points))

    def test_pose_position_validates_rpc_result(self):
        self.assertEqual(
            (1.0, 2.0, -3.0),
            pose_position(
                {"position": {"x_val": 1, "y_val": 2, "z_val": -3}},
                "Drone0",
            ),
        )
        with self.assertRaisesRegex(RuntimeError, "invalid pose"):
            pose_position({}, "Drone0")

    def test_camera_is_updated_from_live_centroid(self):
        class FakeRpc:
            def __init__(self):
                self.camera_call = None
                self.poses = {
                    "Drone0": {"position": {"x_val": 0, "y_val": 0, "z_val": -2}},
                    "Drone1": {"position": {"x_val": 6, "y_val": 0, "z_val": -4}},
                    "Drone2": {"position": {"x_val": 0, "y_val": 6, "z_val": 0}},
                }

            def call(self, method, *args):
                if method == "simGetVehiclePose":
                    return self.poses[args[0]]
                if method == "simSetCameraPose":
                    self.camera_call = args
                    return True
                raise AssertionError(method)

        rpc = FakeRpc()
        center = update_camera(
            rpc,
            ["Drone0", "Drone1", "Drone2"],
            {"name": "swarm_overhead", "altitude_m": 25.0},
        )
        self.assertEqual((2.0, 2.0, -2.0), center)
        name, pose, vehicle, external = rpc.camera_call
        self.assertEqual("swarm_overhead", name)
        self.assertEqual("", vehicle)
        self.assertTrue(external)
        self.assertEqual(
            {"x_val": 2.0, "y_val": 2.0, "z_val": -27.0},
            pose["position"],
        )



class ManagerTrackerDependencyTest(unittest.TestCase):
    def load_manager(self):
        spec = importlib.util.spec_from_file_location(
            "airsim_manager_under_test", ROOT / "simulator/airsim/manager/manager.py"
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_missing_msgpack_fails_before_airsim_starts(self):
        manager = self.load_manager()
        with mock.patch.object(manager.importlib.util, "find_spec", return_value=None):
            with self.assertRaisesRegex(ImportError, "requirements.txt"):
                manager.load_profile(ROOT / "simulator/airsim/scenarios/swarm_open_field/profile.json")


class SwarmOrchestratorTest(unittest.TestCase):
    def test_pc_simulation_matrix_uses_n_independent_onboard_threads(self):
        config = load_config(EXAMPLE)
        command = ["python3", str(FAKE)]
        config["nodes"]["onboard_computer"]["launch_command"] = command
        config["ground_station"]["launch_command"] = command
        config["validation"]["smoke_duration_s"] = 0.2
        with tempfile.TemporaryDirectory() as temporary:
            report = run_matrix(config, Path(temporary), duration_s=0.2)
            self.assertTrue(report["passed"], json.dumps(report, indent=2))
            self.assertEqual(
                ["onboard_center"], report["required_scenarios"]
            )
            for scenario in report["reports"]:
                onboard = [
                    service
                    for service in scenario["services"]
                    if service["identity"].startswith("onboard-")
                ]
                self.assertEqual(3, len(onboard))
                self.assertEqual(
                    3, len({service["supervisor_thread_id"] for service in onboard})
                )
                self.assertEqual(0, scenario["assertions"]["board_vehicle_count"])
                self.assertTrue(scenario["assertions"]["ground_station_on_pc"])

    def test_hybrid_matrix_uses_one_board_and_n_minus_one_containers(self):
        config = load_config(EXAMPLE)
        command = ["python3", str(FAKE)]
        config["deployment"]["stage"] = "hybrid_board"
        config["fleet"]["board_vehicle_index"] = 1
        config["coordination"]["centralized_cases"] = ["board_center", "board_worker"]
        config["nodes"]["board"]["launch_command"] = command
        config["nodes"]["onboard_computer"]["launch_command"] = command
        config["ground_station"]["launch_command"] = command
        with tempfile.TemporaryDirectory() as temporary:
            report = run_matrix(config, Path(temporary), duration_s=0.1)
            self.assertTrue(report["passed"], json.dumps(report, indent=2))
            self.assertEqual(
                ["board_center", "board_worker"], report["required_scenarios"]
            )
            for scenario in report["reports"]:
                placements = [service["placement"] for service in scenario["services"]]
                self.assertEqual(1, placements.count("physical_board"))
                self.assertEqual(2, placements.count("onboard_computer_container"))


if __name__ == "__main__":
    unittest.main()
