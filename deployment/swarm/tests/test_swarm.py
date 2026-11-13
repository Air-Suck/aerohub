import copy
import importlib.util
import json
import math
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
from deployment.swarm.view import camera_options, camera_pose, camera_quaternion, vehicle_centroid, apply_overhead_view


ROOT = Path(__file__).resolve().parents[3]
EXAMPLE = ROOT / "deployment/swarm/config.example.yaml"
FAKE = ROOT / "deployment/swarm/tests/fake_service.py"


class SwarmTopologyTest(unittest.TestCase):
    def setUp(self):
        self.config = load_config(EXAMPLE)

    def test_exactly_one_board_and_remaining_pc(self):
        for count in (2, 3, 6):
            config = copy.deepcopy(self.config)
            config["fleet"]["vehicle_count"] = count
            config["fleet"]["board_vehicle_index"] = count - 1
            topology = resolve_topology(config)
            self.assertEqual(
                1, sum(vehicle["placement"] == "board" for vehicle in topology["vehicles"])
            )
            self.assertEqual(count - 1, len(topology["pc_vehicles"]))
            self.assertTrue(
                all(
                    vehicle["supervisor"] == "independent_thread"
                    for vehicle in topology["pc_vehicles"]
                )
            )

    def test_centralized_matrix_covers_both_board_roles(self):
        topology = resolve_topology(self.config)
        self.assertEqual(
            ["board_center", "board_worker"],
            [scenario["name"] for scenario in topology["scenarios"]],
        )
        board = topology["board_vehicle"]["name"]
        board_worker = topology["scenarios"][1]
        self.assertNotEqual(board, board_worker["center_vehicle"])
        self.assertIn(
            board_worker["center_vehicle"],
            [vehicle["name"] for vehicle in topology["pc_vehicles"]],
        )

    def test_invalid_ground_station_placement_is_rejected(self):
        config = copy.deepcopy(self.config)
        config["ground_station"]["placement"] = "board"
        with self.assertRaisesRegex(ConfigError, "placement must be pc"):
            validate_config(config)

    def test_generated_camera_is_top_down_at_single_and_multi_centroid(self):
        for count in (1, 4):
            config = copy.deepcopy(self.config)
            config["fleet"]["vehicle_count"] = count
            config["fleet"]["board_vehicle_index"] = 0
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

    def test_spawn_relative_poses_are_converted_to_world_before_camera_tracking(self):
        rpc = mock.Mock()
        rpc.call.side_effect = [
            {"position": {"x_val": 1, "y_val": 0, "z_val": -3}},
            {"position": {"x_val": 1, "y_val": 0, "z_val": -3}}, True]
        center = update_camera(rpc, ["Drone0", "Drone1"],
            {"name": "observer", "mode": "oblique", "distance_m": 24., "depression_degrees": 15.},
            {"Drone0": (-50., -9., 0.), "Drone1": (-50., 9., 0.)})
        self.assertEqual((-49., 0., -3.), center)
        self.assertEqual(-73., rpc.call.call_args.args[2]["position"]["x_val"])

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



class ObliqueCameraTest(unittest.TestCase):
    def test_optical_axis_points_at_center_for_multiple_bearings(self):
        center = (-50., 2., -3.)
        for yaw in (0., 90., -135.):
            pose = camera_pose(center, {"mode": "oblique", "distance_m": 24.,
                                        "depression_degrees": 15., "yaw_degrees": yaw})
            q = camera_quaternion(pose)
            w, x, y, z = (q[k] for k in ("w_val", "x_val", "y_val", "z_val"))
            forward = (1 - 2*(y*y + z*z), 2*(x*y + w*z), 2*(x*z - w*y))
            delta = tuple(center[i] - pose[axis] for i, axis in enumerate(("X", "Y", "Z")))
            distance = math.sqrt(sum(v*v for v in delta))
            for actual, expected in zip(forward, delta):
                self.assertAlmostEqual(actual, expected/distance)
            self.assertAlmostEqual(pose["Z"], -3 - 24*math.tan(math.radians(15)))

    def test_initial_and_live_camera_share_geometry(self):
        options = {"name": "observer", "mode": "oblique", "distance_m": 24.,
                   "depression_degrees": 15., "fov_degrees": 60.}
        settings = {"Vehicles": {"Drone0": {"X": -50, "Y": -9},
                                 "Drone1": {"X": -50, "Y": 9}}}
        generated = apply_overhead_view(settings, options)
        rpc = mock.Mock()
        rpc.call.side_effect = [
            {"position": {"x_val": -50, "y_val": -9, "z_val": 0}},
            {"position": {"x_val": -50, "y_val": 9, "z_val": 0}}, True]
        update_camera(rpc, ["Drone0", "Drone1"], options)
        _, pose, _, external = rpc.call.call_args.args[1:]
        initial = generated["ExternalCameras"]["observer"]
        for axis, key in zip(("X", "Y", "Z"), ("x_val", "y_val", "z_val")):
            self.assertEqual(initial[axis], pose["position"][key])
        self.assertEqual(-15., generated["CameraDirector"]["Pitch"])
        self.assertEqual(60., initial["CaptureSettings"][0]["FOV_Degrees"])
        self.assertTrue(external)

    def test_invalid_camera_geometry_is_rejected(self):
        for options in ({"depression_degrees": 91}, {"distance_m": 0},
                        {"yaw_degrees": float("nan")}, {"fov_degrees": 180},
                        {"mode": "unknown"}):
            with self.assertRaises(ValueError):
                camera_options(options)


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
    def test_fake_matrix_uses_independent_pc_threads(self):
        config = load_config(EXAMPLE)
        command = ["python3", str(FAKE)]
        config["nodes"]["board"]["launch_command"] = command
        config["nodes"]["pc"]["launch_command"] = command
        config["ground_station"]["launch_command"] = command
        config["validation"]["smoke_duration_s"] = 0.2
        with tempfile.TemporaryDirectory() as temporary:
            report = run_matrix(config, Path(temporary), duration_s=0.2)
            self.assertTrue(report["passed"], json.dumps(report, indent=2))
            self.assertEqual(
                ["board_center", "board_worker"], report["required_scenarios"]
            )
            for scenario in report["reports"]:
                pc = [
                    service
                    for service in scenario["services"]
                    if service["identity"].startswith("pc-")
                ]
                self.assertEqual(2, len(pc))
                self.assertEqual(2, len({service["supervisor_thread_id"] for service in pc}))
                self.assertTrue(scenario["assertions"]["ground_station_on_pc"])


if __name__ == "__main__":
    unittest.main()
