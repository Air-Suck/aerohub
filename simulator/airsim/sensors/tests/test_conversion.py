import math
import struct
import unittest
from uav_as2_sensor_adapter.conversion import (
    extrinsic, image_payload, imu_payload, lidar_payload, multiply)


class ConversionTests(unittest.TestCase):
    def test_camera_optical_axes(self):
        _, q = extrinsic({}, optical=True)
        qi = (-q[0], -q[1], -q[2], q[3])
        # Optical forward maps to body forward; optical right maps to body right.
        self.assertEqual(multiply(multiply(q, (0, 0, 1, 0)), qi)[:3], (1, 0, 0))
        self.assertEqual(multiply(multiply(q, (1, 0, 0, 0)), qi)[:3], (0, -1, 0))

    def test_mount_translation(self):
        self.assertEqual(extrinsic({'X': 1, 'Y': 2, 'Z': 3})[0], (1, -2, -3))

    def test_depth_is_float_meters(self):
        encoding, step, data = image_payload({'width': 2, 'height': 1, 'image_data_float': [1.25, 3]}, 1)
        self.assertEqual((encoding, step), ('32FC1', 8))
        self.assertEqual(struct.unpack('<ff', data), (1.25, 3))

    def test_malformed_rgb_rejected(self):
        with self.assertRaises(ValueError):
            image_payload({'width': 2, 'height': 2, 'image_data_uint8': b'bad'}, 0)

    def test_bgr_normalized_to_rgb(self):
        encoding, _, data = image_payload({'width': 1, 'height': 1,
                                         'image_data_uint8': bytes([3, 2, 1])}, 0, 'bgr')
        self.assertEqual((encoding, data), ('rgb8', bytes([1, 2, 3])))

    def test_world_cloud_becomes_sensor_local_flu(self):
        s = math.sqrt(0.5)
        response = {'point_cloud': [10, 21, 30], 'pose': {
            'position': dict(x_val=10, y_val=20, z_val=30),
            'orientation': dict(x_val=0, y_val=0, z_val=s, w_val=s)}}
        point = struct.unpack('<fff', lidar_payload(response, 'VehicleInertialFrame'))
        self.assertAlmostEqual(point[0], 1)
        self.assertAlmostEqual(point[1], 0)
        self.assertAlmostEqual(point[2], 0)

    def test_sensor_cloud_frd_to_flu(self):
        response = {'point_cloud': [1, 2, 3], 'pose': {
            'position': dict(x_val=0, y_val=0, z_val=0),
            'orientation': dict(x_val=0, y_val=0, z_val=0, w_val=1)}}
        self.assertEqual(struct.unpack('<fff', lidar_payload(response, 'SensorLocalFrame')), (1, -2, -3))

    def test_imu_frd_to_flu(self):
        response = {
            'orientation': dict(x_val=0, y_val=0, z_val=0, w_val=2),
            'angular_velocity': dict(x_val=1, y_val=2, z_val=3),
            'linear_acceleration': dict(x_val=4, y_val=5, z_val=6),
        }
        orientation, angular, acceleration = imu_payload(response)
        self.assertEqual(orientation, (0, 0, 0, 1))
        self.assertEqual(angular, (1, -2, -3))
        self.assertEqual(acceleration, (4, -5, -6))

    def test_nonfinite_imu_rejected(self):
        response = {
            'orientation': dict(x_val=0, y_val=0, z_val=0, w_val=1),
            'angular_velocity': dict(x_val=0, y_val=float('nan'), z_val=0),
            'linear_acceleration': dict(x_val=0, y_val=0, z_val=0),
        }
        with self.assertRaises(ValueError):
            imu_payload(response)
