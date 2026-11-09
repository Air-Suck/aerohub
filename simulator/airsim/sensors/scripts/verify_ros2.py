"""Live ROS subscriber acceptance, independent of AirSim RPC."""
import json
import math
import struct
import time
import os

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy


def main():
    with open(os.environ.get('AIRSIM_SETTINGS', '/config/settings.json')) as stream:
        vehicle = json.load(stream)['Vehicles']['PX4']
    rclpy.init()
    node = rclpy.create_node('verify_platform_sensors')
    expected, received, subscriptions = {}, {}, []
    kinds = {0: 'rgb', 1: 'depth_planar', 2: 'depth_perspective', 5: 'segmentation'}
    frames = {'drone0/' + name + '_optical' for name in vehicle.get('Cameras', {})}
    frames.update('drone0/' + name for name, c in vehicle.get('Sensors', {}).items() if c['SensorType'] == 6)
    tf_frames = set()

    def on_tf(msg):
        tf_frames.update(t.child_frame_id for t in msg.transforms if t.header.frame_id == 'drone0/base_link')

    # Aggregate latched transforms from every static broadcaster. A depth of one
    # can retain only the state estimator sample and discard sensor extrinsics.
    subscriptions.append(node.create_subscription(TFMessage, '/tf_static', on_tf,
        QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)))

    def subscribe(topic, kind):
        expected[topic] = kind

        def callback(msg):
            assert msg.header.stamp.sec > 0 and msg.header.frame_id in frames
            if kind is Image:
                assert msg.width > 0 and msg.height > 0
                assert len(msg.data) == msg.height * msg.step
                received[topic] = dict(width=msg.width, height=msg.height, encoding=msg.encoding)
            elif kind is PointCloud2:
                assert msg.point_step == 12
                assert len(msg.data) == msg.row_step * msg.height
                if not msg.width:
                    return
                assert all(math.isfinite(v) for v in struct.unpack_from('<fff', bytes(msg.data)))
                received[topic] = dict(points=msg.width)
            else:
                assert msg.k[0] > 0 and msg.width > 0
                received[topic] = dict(width=msg.width, height=msg.height)
        subscriptions.append(node.create_subscription(kind, topic, callback, qos_profile_sensor_data))

    for name, config in vehicle.get('Cameras', {}).items():
        for capture in config['CaptureSettings']:
            prefix = '/drone0/sensor_measurements/' + name + '/' + kinds[capture['ImageType']]
            subscribe(prefix + '/image', Image)
            subscribe(prefix + '/camera_info', CameraInfo)
    for name, config in vehicle.get('Sensors', {}).items():
        if config['SensorType'] == 6:
            subscribe('/drone0/sensor_measurements/' + name + '/points', PointCloud2)
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline and (set(received) != set(expected) or not frames <= tf_frames):
        rclpy.spin_once(node, timeout_sec=0.2)
    missing = sorted(set(expected) - set(received))
    print(json.dumps(dict(received=received, missing=missing, missing_tf=sorted(frames-tf_frames)), indent=2))
    node.destroy_node()
    rclpy.shutdown()
    if missing or not frames <= tf_frames:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
