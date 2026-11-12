import collections
import math
import threading

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from .alignment import estimate_similarity
from .engine import DpvSlamEngine
from .motion import matrix_from_pose, quaternion_xyzw


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


class DpvSlamNode(Node):
    def __init__(self):
        super().__init__("dpv_slam")
        for name, default in {
            "image_topic": "sensor_measurements/FrontCamera/rgb/image",
            "camera_info_topic": "sensor_measurements/FrontCamera/rgb/camera_info",
            "local_odometry_topic": "sensor_measurements/odom",
            "expected_camera_frame": "drone0/FrontCamera_optical",
            "map_frame": "dpv_slam_map",
            "camera_frame": "dpv_slam_camera",
            "config_path": "",
            "network_path": "",
        }.items():
            self.declare_parameter(name, default)
        for name, default in {
            "pairing_tolerance": 0.05,
            "minimum_alignment_baseline": 1.0,
            "minimum_alignment_horizontal_baseline": 1.0,
            "minimum_alignment_horizontal_excitation": 0.2,
            "alignment_rmse_max": 0.35,
            "scale_min": 0.05,
            "scale_max": 20.0,
        }.items():
            self.declare_parameter(name, default)
        self.declare_parameter("alignment_window", 120)
        self.declare_parameter("alignment_freeze_samples", 30)
        self.declare_parameter("map_max_points", 20000)
        self._engine = DpvSlamEngine(
            self.get_parameter("config_path").value,
            self.get_parameter("network_path").value)
        self._expected_frame = self.get_parameter("expected_camera_frame").value
        self._map_frame = self.get_parameter("map_frame").value
        self._camera_frame = self.get_parameter("camera_frame").value
        self._tolerance_ns = int(self.get_parameter("pairing_tolerance").value * 1e9)
        self._window = int(self.get_parameter("alignment_window").value)
        self._freeze_samples = int(self.get_parameter("alignment_freeze_samples").value)
        self._map_max_points = int(self.get_parameter("map_max_points").value)
        if (self._window < 5 or self._freeze_samples < 5 or
                self._freeze_samples > self._window or self._map_max_points <= 0 or
                self._tolerance_ns <= 0):
            raise RuntimeError("alignment and map limits must be positive")
        self._camera_info = {}
        self._images = {}
        self._local = collections.deque(maxlen=max(400, self._window * 4))
        self._visual_positions = collections.deque(maxlen=self._window)
        self._metric_positions = collections.deque(maxlen=self._window)
        self._alignment = None
        self._last_image_stamp = -1
        self._last_result_stamp = -1
        self._path = Path()
        self._path.header.frame_id = self._map_frame
        self._busy = threading.Lock()
        self._state_lock = threading.Lock()
        self._callbacks = ReentrantCallbackGroup()
        self._odom_pub = self.create_publisher(Odometry, "dpv_slam/odometry", 10)
        self._path_pub = self.create_publisher(Path, "dpv_slam/path", 10)
        self._map_pub = self.create_publisher(PointCloud2, "dpv_slam/sparse_points", 2)
        self._status_pub = self.create_publisher(DiagnosticArray, "dpv_slam/status", 10)
        self.create_subscription(CameraInfo, self.get_parameter("camera_info_topic").value,
                                 self._on_info, qos_profile_sensor_data,
                                 callback_group=self._callbacks)
        self.create_subscription(Odometry, self.get_parameter("local_odometry_topic").value,
                                 self._on_local, qos_profile_sensor_data,
                                 callback_group=self._callbacks)
        self.create_subscription(Image, self.get_parameter("image_topic").value,
                                 self._on_image, qos_profile_sensor_data,
                                 callback_group=self._callbacks)

    def _status(self, level, message, **values):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        item = DiagnosticStatus(level=level, name="dpv_slam/runtime", message=message)
        item.values = [KeyValue(key=str(key), value=str(value)) for key, value in values.items()]
        array.status = [item]
        self._status_pub.publish(array)

    def _on_info(self, message):
        key = stamp_ns(message)
        if key > 0 and message.header.frame_id == self._expected_frame:
            with self._state_lock:
                self._camera_info[key] = message
                while len(self._camera_info) > 40:
                    self._camera_info.pop(next(iter(self._camera_info)))
            self._try_process(key)

    def _on_local(self, message):
        key = stamp_ns(message)
        if key > 0:
            pose = matrix_from_pose(message.pose.pose)
            with self._state_lock:
                self._local.append((key, pose))

    def _nearest_local(self, key):
        with self._state_lock:
            if not self._local:
                return None
            nearest = min(self._local, key=lambda item: abs(item[0] - key))
        return nearest[1] if abs(nearest[0] - key) <= self._tolerance_ns else None

    @staticmethod
    def _rgb(message):
        if message.encoding not in ("rgb8", "bgr8") or message.step < message.width * 3:
            raise ValueError("DPV-SLAM requires packed rgb8 or bgr8")
        data = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        image = data[:, :message.width * 3].reshape(message.height, message.width, 3).copy()
        return image if message.encoding == "rgb8" else image[..., ::-1]

    def _on_image(self, message):
        key = stamp_ns(message)
        with self._state_lock:
            invalid_stamp = key <= self._last_image_stamp
        if invalid_stamp or message.header.frame_id != self._expected_frame:
            self._status(DiagnosticStatus.WARN, "input_rejected", reason="frame_or_stamp")
            return
        with self._state_lock:
            self._images[key] = message
            while len(self._images) > 10:
                self._images.pop(next(iter(self._images)))
        self._try_process(key)

    def _try_process(self, key):
        with self._state_lock:
            if key <= self._last_image_stamp:
                self._images.pop(key, None)
                self._camera_info.pop(key, None)
                return
            message = self._images.get(key)
            info = self._camera_info.get(key)
            if message is None or info is None:
                return
            self._images.pop(key)
            self._camera_info.pop(key)
            self._last_image_stamp = key
        local = self._nearest_local(key)
        if local is None:
            self._status(DiagnosticStatus.WARN, "input_rejected", reason="unpaired_local_state")
            return
        self._process_image(message, info, key)

    def _process_image(self, message, info, key):
        if not self._busy.acquire(blocking=False):
            self._status(DiagnosticStatus.WARN, "input_dropped", reason="algorithm_busy")
            return
        try:
            intrinsics = np.array([info.k[0], info.k[4], info.k[2], info.k[5]], dtype=np.float32)
            if not np.isfinite(intrinsics).all() or np.min(intrinsics[:2]) <= 0:
                raise ValueError("invalid camera intrinsics")
            result = self._engine.process(key, self._rgb(message), intrinsics)
            if not result.initialized:
                self._status(DiagnosticStatus.OK, "initializing", frames=result.frame_count)
                return
            if result.stamp_ns <= self._last_result_stamp:
                self._status(DiagnosticStatus.OK, "tracking_no_new_keyframe",
                             frames=result.frame_count)
                return
            result_local = self._nearest_local(result.stamp_ns)
            if result_local is None:
                self._status(DiagnosticStatus.WARN, "input_rejected",
                             reason="unpaired_keyframe_local_state")
                return
            self._last_result_stamp = result.stamp_ns
            if self._alignment is None:
                self._visual_positions.append(result.pose[:3, 3].copy())
                self._metric_positions.append(result_local[:3, 3].copy())
                if len(self._visual_positions) < self._freeze_samples:
                    self._status(DiagnosticStatus.OK, "aligning_scale",
                                 samples=len(self._visual_positions))
                    return
                try:
                    candidate = estimate_similarity(
                        self._visual_positions, self._metric_positions,
                        minimum_baseline=self.get_parameter(
                            "minimum_alignment_baseline").value,
                        minimum_horizontal_baseline=self.get_parameter(
                            "minimum_alignment_horizontal_baseline").value,
                        minimum_horizontal_excitation=self.get_parameter(
                            "minimum_alignment_horizontal_excitation").value,
                        scale_bounds=(self.get_parameter("scale_min").value,
                                      self.get_parameter("scale_max").value))
                except ValueError as error:
                    self._status(DiagnosticStatus.OK, "aligning_scale", reason=str(error))
                    return
                if candidate.rmse > self.get_parameter("alignment_rmse_max").value:
                    self._status(DiagnosticStatus.ERROR, "alignment_rejected",
                                 rmse=candidate.rmse)
                    return
                self._alignment = candidate
            alignment = self._alignment
            pose = alignment.pose(result.pose)
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(
                result.stamp_ns, 1_000_000_000)
            self._publish_state(message, pose)
            self._publish_points(message, alignment, result.points)
            self._status(DiagnosticStatus.OK, "tracking", frames=result.frame_count,
                         scale=alignment.scale, alignment_rmse=alignment.rmse,
                         sparse_points=0 if result.points is None else len(result.points))
        except Exception as error:
            self.get_logger().error(f"DPV-SLAM processing failed: {error!r}")
            self._status(DiagnosticStatus.ERROR, "processing_failed", error=repr(error))
        finally:
            self._busy.release()

    def _publish_state(self, source, pose):
        quaternion = quaternion_xyzw(pose[:3, :3])
        odometry = Odometry()
        odometry.header.stamp = source.header.stamp
        odometry.header.frame_id = self._map_frame
        odometry.child_frame_id = self._camera_frame
        odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z = pose[:3, 3]
        odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y = quaternion[:2]
        odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w = quaternion[2:]
        self._odom_pub.publish(odometry)
        item = PoseStamped(header=odometry.header, pose=odometry.pose.pose)
        self._path.header.stamp = source.header.stamp
        self._path.poses.append(item)
        self._path_pub.publish(self._path)

    def _publish_points(self, source, alignment, points):
        if points is None or not len(points):
            return
        stride = max(1, math.ceil(len(points) / self._map_max_points))
        selected = points[::stride]
        selected = (alignment.scale * (alignment.rotation @ selected.T)).T + alignment.translation
        header = Header(stamp=source.header.stamp, frame_id=self._map_frame)
        self._map_pub.publish(point_cloud2.create_cloud_xyz32(header, selected.astype(np.float32)))


def main(args=None):
    rclpy.init(args=args)
    node = DpvSlamNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
