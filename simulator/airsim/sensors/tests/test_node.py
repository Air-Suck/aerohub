import json
from pathlib import Path
import tempfile
import unittest

try:
    import rclpy
    from uav_as2_sensor_adapter.node import SensorNode
except ModuleNotFoundError as error:
    if error.name != 'rclpy':
        raise
    rclpy = None


@unittest.skipIf(rclpy is None, 'requires the ROS runtime')
class SensorNodeTests(unittest.TestCase):
    def check_workers(self, camera_enabled, expected, camera_parallelism=1):
        with tempfile.TemporaryDirectory() as temporary:
            settings = Path(temporary) / 'settings.json'
            settings.write_text(json.dumps({'Vehicles': {'PX4': {
                'Cameras': {'Front': {'CaptureSettings': [{'ImageType': 0}, {'ImageType': 2}]}},
                'Sensors': {
                    'imu': {'SensorType': 2, 'Enabled': True},
                    'Lidar': {'SensorType': 6, 'Enabled': True},
                }
            }}}))
            rclpy.init(args=['--ros-args', '-p', 'settings_file:=' + str(settings),
                            '-p', 'camera_enabled:=' + str(camera_enabled).lower(),
                            '-p', 'camera_parallelism:=' + str(camera_parallelism)])
            node = None
            workers = []
            try:
                node = SensorNode()
                workers = list(node.workers)
                self.assertEqual({worker.name for worker in workers}, expected)
                self.assertEqual(len(node.rpc_clients), len(expected))
                self.assertTrue(all(worker.is_alive() for worker in workers))
                # Timers have not run: idle threads must not make any RPC calls.
                self.assertEqual(node.last_stamp, {})
            finally:
                if node is not None:
                    node.destroy_node()
                rclpy.try_shutdown()
            self.assertTrue(all(not worker.is_alive() for worker in workers))

    def test_rgb_and_depth_share_one_camera_worker(self):
        self.check_workers(True, {'camera-Front', 'imu-imu', 'lidar-Lidar'})

    def test_camera_disabled_keeps_independent_lidar_worker(self):
        self.check_workers(False, {'imu-imu', 'lidar-Lidar'})

    def test_parallel_camera_workers_have_independent_rpc_clients(self):
        self.check_workers(
            True,
            {'camera-Front-0', 'camera-Front-1', 'imu-imu', 'lidar-Lidar'},
            camera_parallelism=2)
