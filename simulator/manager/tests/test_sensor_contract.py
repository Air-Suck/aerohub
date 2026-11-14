from copy import deepcopy
import unittest

from simulator.projectairsim.sensor_contract import streams
from uav_as2_sensor_adapter.conversion import multiply


class SensorContractTest(unittest.TestCase):
    def setUp(self):
        self.camera = {"id": "front", "type": "camera", "enabled": True,
            "parent-link": "Frame", "origin": {"xyz": "1 2 3", "rpy-deg": "0 0 90"},
            "capture-settings": [{"image-type": k, "capture-enabled": True}
                                 for k in (0, 1, 2)]}
        self.scene = {"id": "Scene", "actors": [{"name": "Drone1", "type": "robot",
            "robot-config": {"links": [{"name": "Frame"}], "sensors": [self.camera]}}]}
        self.runtime = {"vehicles": [{"name": "Drone1", "namespace": "drone0"}]}

    def test_distinct_depth_models_and_nonidentity_optical_transform(self):
        original = deepcopy(self.scene)
        inventory = streams(self.scene, self.runtime)
        self.assertIn("/depth_planar/", inventory[1]["target"])
        self.assertIn("/depth_perspective/", inventory[2]["target"])
        xyz, q = inventory[0]["transform"]
        self.assertEqual(xyz, (1, -2, -3))
        # Camera optical +Z points along native body +Y at yaw90, i.e. ROS -Y.
        v = multiply(multiply(q, (0, 0, 1, 0)), (-q[0], -q[1], -q[2], q[3]))
        self.assertAlmostEqual(v[0], 0, places=6)
        self.assertAlmostEqual(v[1], -1, places=6)
        self.assertAlmostEqual(v[2], 0, places=6)
        self.assertEqual(self.scene, original)

    def test_reject_stabilized_camera_instead_of_static_tf(self):
        self.camera["gimbal"] = {"lock-roll": True}
        with self.assertRaisesRegex(ValueError, "dynamic TF"):
            streams(self.scene, self.runtime)

    def test_reject_imu_pose_ignored_by_native_model(self):
        self.camera.update(type="imu")
        with self.assertRaisesRegex(ValueError, "IMU extrinsics"):
            streams(self.scene, self.runtime)

    def test_reject_joint_sensor(self):
        self.camera["parent-link"] = "gimbal"
        with self.assertRaisesRegex(ValueError, "root-link"):
            streams(self.scene, self.runtime)


if __name__ == "__main__":
    unittest.main()
