"""AirSim backend for the platform's ROS sensor contract."""
import json
from array import array
import math
import re
import time
from functools import partial
from itertools import cycle
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .conversion import extrinsic, image_payload, imu_payload, lidar_payload
from .rpc import Rpc
from .worker import SensorWorker


IMAGE_NAMES = {0: 'rgb', 1: 'depth_planar', 2: 'depth_perspective', 5: 'segmentation'}


def identifier(value):
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', value):
        raise ValueError('sensor/namespace names must be ROS identifiers: ' + value)
    return value


class SensorNode(Node):
    def __init__(self):
        super().__init__('platform_sensor_adapter')
        self.declare_parameter('settings_file', '')
        self.declare_parameter('vehicle_name', 'PX4')
        self.declare_parameter('vehicle_namespace', 'drone0')
        self.declare_parameter('rpc_host', '127.0.0.1')
        self.declare_parameter('rpc_port', 41471)
        self.declare_parameter('camera_rate_hz', 5.0)
        self.declare_parameter('camera_parallelism', 1)
        self.declare_parameter('camera_enabled', True)
        self.declare_parameter('camera_reliable', False)
        self.declare_parameter('imu_rate_hz', 100.0)
        self.declare_parameter('lidar_rate_hz', 10.0)
        self.declare_parameter('rpc_timeout', 3.0)
        self.declare_parameter('source_color_order', 'rgb')
        self.declare_parameter('timing_file', '')
        camera_qos = (
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
            if self.get_parameter('camera_reliable').value
            else qos_profile_sensor_data)
        timing_path = self.get_parameter('timing_file').value
        self.timing = open(timing_path, 'w', buffering=1) if timing_path else None
        self.color_order = self.get_parameter('source_color_order').value
        if self.color_order not in ('rgb', 'bgr'):
            raise ValueError('source_color_order must be rgb or bgr')
        self.vehicle = self.get_parameter('vehicle_name').value
        ns = identifier(self.get_parameter('vehicle_namespace').value)
        self.prefix = '/' + ns + '/sensor_measurements'
        self.body_frame = ns + '/base_link'
        self.host = self.get_parameter('rpc_host').value
        self.port = self.get_parameter('rpc_port').value
        self.timeout = self.get_parameter('rpc_timeout').value
        with open(self.get_parameter('settings_file').value) as stream:
            settings = json.load(stream)
        vehicle = settings['Vehicles'][self.vehicle]
        self.static_tf = StaticTransformBroadcaster(self)
        transforms = []
        self.streams = []
        self.workers = []
        self.rpc_clients = []
        self.last_stamp = {}
        self.errors = {}
        for name, camera in vehicle.get('Cameras', {}).items():
            identifier(name)
            config = dict(settings.get('CameraDefaults', {}))
            config.update(camera)
            frame = ns + '/' + name + '_optical'
            transforms.append(self.transform(frame, config, optical=True))
            if not self.get_parameter('camera_enabled').value:
                continue
            requests, pubs, captures = [], [], []
            seen = set()
            for capture in config.get('CaptureSettings', []):
                kind = int(capture['ImageType'])
                if kind not in IMAGE_NAMES:
                    self.get_logger().warning(f'{name}: skipping unsupported ImageType {kind}')
                    continue
                if kind in seen:
                    raise ValueError('duplicate camera ImageType')
                seen.add(kind)
                requests.append(dict(camera_name=name, image_type=kind,
                                     pixels_as_float=kind in (1, 2), compress=False))
                topic = self.prefix + '/' + name + '/' + IMAGE_NAMES[kind]
                pubs.append((self.create_publisher(Image, topic + '/image', camera_qos),
                             self.create_publisher(CameraInfo, topic + '/camera_info', camera_qos)))
                captures.append(capture)
            if requests:
                parallelism = int(self.get_parameter('camera_parallelism').value)
                if not 1 <= parallelism <= 4:
                    raise ValueError('camera_parallelism must be between 1 and 4')
                callback = partial(
                    self.camera, name, frame, requests, pubs, captures, Lock())
                self.timer(
                    'camera_rate_hz', callback, 'camera-' + name, parallelism)
        for name, config in vehicle.get('Sensors', {}).items():
            if not config.get('Enabled', True):
                continue
            identifier(name)
            frame = ns + '/' + name
            sensor_type = config.get('SensorType')
            if sensor_type == 2:
                transforms.append(self.transform(frame, config))
                pub = self.create_publisher(
                    Imu, self.prefix + '/' + name + '/data', qos_profile_sensor_data)
                self.timer('imu_rate_hz', partial(self.imu, name, frame, pub), 'imu-' + name)
                continue
            if sensor_type != 6:
                continue
            data_frame = config.get('DataFrame', 'VehicleInertialFrame')
            if data_frame not in ('VehicleInertialFrame', 'SensorLocalFrame'):
                raise ValueError('unsupported LiDAR DataFrame')
            transforms.append(self.transform(frame, config))
            pub = self.create_publisher(PointCloud2, self.prefix + '/' + name + '/points', qos_profile_sensor_data)
            self.timer('lidar_rate_hz', partial(self.lidar, name, frame, data_frame, pub), 'lidar-' + name)
        self.static_tf.sendTransform(transforms)
        if not self.streams:
            raise ValueError('no supported sensors configured')
        for worker in self.workers:
            worker.start()
        if self.timing:
            self.timing.write(json.dumps(dict(kind='inventory',workers=[w.name for w in self.workers]))+'\n')

    def timer(self, parameter, callback, name, parallelism=1):
        rate = float(self.get_parameter(parameter).value)
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError(parameter + ' must be positive and finite')
        workers = []
        for index in range(parallelism):
            rpc = Rpc(self.host, self.port, self.timeout)
            worker_name = name if parallelism == 1 else f'{name}-{index}'
            worker = SensorWorker(
                worker_name, partial(self.guarded, callback, rpc))
            self.workers.append(worker)
            self.rpc_clients.append(rpc)
            workers.append(worker)
        requests = cycle(workers)
        self.streams.append(self.create_timer(
            1.0 / rate, lambda: next(requests).request()))

    def destroy_node(self):
        # Stop all producers before destroying publishers/clock used by their callbacks.
        for timer in self.streams:
            timer.cancel()
        for worker in self.workers:
            worker.stop()
        for rpc in self.rpc_clients:
            rpc.close()
        for worker in self.workers:
            worker.join()
        if self.timing:
            self.timing.close()
        return super().destroy_node()

    def guarded(self, callback, rpc):
        try:
            callback(rpc)
        except Exception as exc:
            if not rclpy.ok(context=self.context):
                return
            key = str(callback)
            now = self.get_clock().now().nanoseconds
            if now - self.errors.get(key, -10**30) > 5_000_000_000:
                self.get_logger().error(f'sensor read failed: {exc}')
                self.errors[key] = now

    def transform(self, frame, config, optical=False):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = self.body_frame, frame
        position, rotation = extrinsic(config, optical)
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = position
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = rotation
        return msg

    def header(self, msg, frame, timestamp):
        timestamp = int(timestamp)
        if timestamp <= 0:
            raise ValueError('invalid sensor timestamp')
        msg.header.frame_id = frame
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(timestamp, 1_000_000_000)

    def camera(self, name, frame, requests, publishers, captures, lock, rpc):
        started = time.monotonic()
        sim_started = self.get_clock().now().nanoseconds
        responses = rpc.call('simGetImages', requests, self.vehicle, False)
        acquired = time.monotonic()
        sim_acquired = self.get_clock().now().nanoseconds
        if len(responses) != len(requests):
            raise ValueError('incomplete image response')
        converted_responses = []
        for request, response, publishers_for_image, capture in zip(
                requests, responses, publishers, captures):
            encoding, step, data = image_payload(response, request['image_type'], self.color_order)
            converted_responses.append((
                request, response, publishers_for_image, capture,
                encoding, step, data, time.monotonic()))
        with lock:
            for (request, response, (pub, info_pub), capture,
                 encoding, step, data, conversion_finished) in converted_responses:
                key = (name, request['image_type'])
                timestamp = int(response['time_stamp'])
                if timestamp <= self.last_stamp.get(key, 0):
                    continue
                msg = Image()
                self.header(msg, frame, timestamp)
                msg.width, msg.height = response['width'], response['height']
                # Generated ROS setters validate bytes element by element. A typed
                # uint8 array keeps the identical payload without that Python loop.
                msg.encoding, msg.step, msg.data = encoding, step, array('B', data)
                fov = float(capture.get('FOV_Degrees', 90))
                if not math.isfinite(fov) or not 0 < fov < 180:
                    raise ValueError('invalid camera FOV')
                focal = msg.width / (2 * math.tan(math.radians(fov) / 2))
                cx, cy = (msg.width - 1) / 2, (msg.height - 1) / 2
                info = CameraInfo()
                info.header = msg.header
                info.width, info.height = msg.width, msg.height
                info.distortion_model, info.d = 'plumb_bob', [0.0] * 5
                info.k = [focal, 0., cx, 0., focal, cy, 0., 0., 1.]
                info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
                info.p = [focal, 0., cx, 0., 0., focal, cy, 0., 0., 0., 1., 0.]
                pub.publish(msg)
                info_pub.publish(info)
                self.last_stamp[key] = timestamp
                if self.timing:
                    self.timing.write(json.dumps(dict(
                        kind='camera', camera=name,
                        image_type=request['image_type'], stamp_ns=timestamp,
                        wall=started, sim_start_ns=sim_started,
                        sim_acquired_ns=sim_acquired,
                        rpc_wall_s=acquired - started,
                        conversion_wall_s=conversion_finished - acquired,
                        publish_wall_s=time.monotonic() - conversion_finished,
                        bytes=len(data))) + '\n')

    def lidar(self, name, frame, data_frame, publisher, rpc):
        response = rpc.call('getLidarData', name, self.vehicle)
        timestamp = int(response['time_stamp'])
        if self.last_stamp.get(name) == timestamp:
            return
        msg = PointCloud2()
        self.header(msg, frame, timestamp)
        msg.data = lidar_payload(response, data_frame)
        msg.height, msg.width = 1, len(msg.data) // 12
        msg.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(('x', 'y', 'z'))]
        msg.point_step, msg.row_step = 12, len(msg.data)
        msg.is_dense = False
        publisher.publish(msg)
        self.last_stamp[name] = timestamp

    def imu(self, name, frame, publisher, rpc):
        response = rpc.call('getImuData', name, self.vehicle)
        timestamp = int(response['time_stamp'])
        key = (name, 'imu')
        if self.last_stamp.get(key) == timestamp:
            return
        msg = Imu()
        self.header(msg, frame, timestamp)
        orientation, angular, acceleration = imu_payload(response)
        (msg.orientation.x, msg.orientation.y, msg.orientation.z,
         msg.orientation.w) = orientation
        (msg.angular_velocity.x, msg.angular_velocity.y,
         msg.angular_velocity.z) = angular
        (msg.linear_acceleration.x, msg.linear_acceleration.y,
         msg.linear_acceleration.z) = acceleration
        publisher.publish(msg)
        self.last_stamp[key] = timestamp


def main():
    rclpy.init()
    node = None
    try:
        node = SensorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
