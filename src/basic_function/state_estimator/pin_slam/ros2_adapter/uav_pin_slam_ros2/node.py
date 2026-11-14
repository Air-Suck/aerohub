import os
import threading
from pathlib import Path

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry, Path as PathMessage
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from .contracts import PointCloudContract, xyz_fields_valid
from .engine import PinSlamEngine
from .motion import body_twist, quaternion_xyzw


class PinSlamNode(Node):
    def __init__(self) -> None:
        super().__init__("pin_slam")
        self.declare_parameter("input_topic", "sensor_measurements/Lidar/points")
        self.declare_parameter("expected_input_frame", "drone0/Lidar")
        self.declare_parameter("map_frame", "pin_slam_map")
        self.declare_parameter("sensor_frame", "pin_slam_lidar")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_root", "/tmp/pin_slam")
        self.declare_parameter("map_max_points", 500000)
        self.declare_parameter("map_publish_every_n_frames", 10)
        self.declare_parameter("input_queue_depth", 20)
        config_path = self.get_parameter("config_path").value
        upstream_root = os.environ.get("PIN_SLAM_ROOT", "")
        if not config_path or not Path(config_path).is_file():
            raise RuntimeError("config_path must identify the deployed PIN-SLAM YAML")
        self._engine = PinSlamEngine(
            upstream_root, config_path, self.get_parameter("output_root").value)
        self._contract = PointCloudContract(self.get_parameter("expected_input_frame").value)
        self._map_frame = self.get_parameter("map_frame").value
        self._sensor_frame = self.get_parameter("sensor_frame").value
        self._map_max_points = int(self.get_parameter("map_max_points").value)
        if self._map_max_points <= 0:
            raise RuntimeError("map_max_points must be positive")
        self._map_publish_every_n_frames = int(
            self.get_parameter("map_publish_every_n_frames").value)
        if self._map_publish_every_n_frames <= 0:
            raise RuntimeError("map_publish_every_n_frames must be positive")
        input_queue_depth = int(self.get_parameter("input_queue_depth").value)
        if input_queue_depth <= 0:
            raise RuntimeError("input_queue_depth must be positive")
        self._processed_input_count = 0
        self._previous_pose = None
        self._previous_stamp_ns = None
        self._path = PathMessage()
        self._path.header.frame_id = self._map_frame
        self._lock = threading.Lock()

        self._odom_pub = self.create_publisher(Odometry, "pin_slam/odometry", 10)
        self._path_pub = self.create_publisher(PathMessage, "pin_slam/path", 10)
        self._map_pub = self.create_publisher(PointCloud2, "pin_slam/neural_points", 2)
        self._status_pub = self.create_publisher(DiagnosticArray, "pin_slam/status", 10)
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=input_queue_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            PointCloud2, self.get_parameter("input_topic").value,
            self._on_cloud, input_qos)
        self.create_service(Trigger, "pin_slam/save_results", self._save_results)
        self.create_service(Trigger, "pin_slam/save_mesh", self._save_mesh)

    @staticmethod
    def _stamp_ns(message: PointCloud2) -> int:
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def _status(self, level: int, message: str, **values) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        item = DiagnosticStatus(level=level, name="pin_slam/runtime", message=message)
        item.values = [KeyValue(key=str(k), value=str(v)) for k, v in values.items()]
        array.status = [item]
        self._status_pub.publish(array)

    def _on_cloud(self, message: PointCloud2) -> None:
        stamp_ns = self._stamp_ns(message)
        point_count = int(message.width) * int(message.height)
        if not xyz_fields_valid(message.fields):
            self._status(DiagnosticStatus.WARN, "input_rejected", reason="invalid_xyz_fields")
            return
        decision = self._contract.check(message.header.frame_id, stamp_ns, point_count)
        if not decision.accepted:
            self._status(DiagnosticStatus.WARN, "input_rejected", reason=decision.reason)
            return
        if not self._lock.acquire(blocking=False):
            self._status(DiagnosticStatus.WARN, "input_dropped", reason="algorithm_busy")
            return
        try:
            publish_map = (
                self._processed_input_count % self._map_publish_every_n_frames == 0)
            result = self._engine.process(
                message, self._map_max_points, publish_map=publish_map)
            self._processed_input_count += 1
            if not result.tracking_ok:
                self._status(DiagnosticStatus.ERROR, "tracking_lost", frame=result.processed_frame)
                self._previous_pose = None
                self._previous_stamp_ns = None
                return
            if result.loop_corrected:
                self._previous_pose = result.pose
                self._previous_stamp_ns = stamp_ns
                self._status(
                    DiagnosticStatus.WARN, "loop_corrected_twist_reset",
                    frame=result.processed_frame)
                return
            if self._previous_pose is None:
                self._previous_pose = result.pose
                self._previous_stamp_ns = stamp_ns
                self._status(DiagnosticStatus.OK, "initializing_twist", frame=result.processed_frame)
                return
            dt = (stamp_ns - self._previous_stamp_ns) * 1.0e-9
            twist = body_twist(self._previous_pose, result.pose, dt)
            self._publish_state(message, result.pose, twist)
            self._publish_map(message, result.neural_points)
            self._previous_pose = result.pose
            self._previous_stamp_ns = stamp_ns
            self._status(
                DiagnosticStatus.OK, "tracking", frame=result.processed_frame,
                map_points=0 if result.neural_points is None else len(result.neural_points),
                loop_corrected=result.loop_corrected)
        except Exception as error:
            self._status(DiagnosticStatus.ERROR, "processing_failed", error=repr(error))
            self.get_logger().error(f"PIN-SLAM processing failed: {error!r}")
            self._previous_pose = None
            self._previous_stamp_ns = None
        finally:
            self._lock.release()

    def _publish_state(self, source: PointCloud2, pose: np.ndarray, twist) -> None:
        quaternion = quaternion_xyzw(pose[:3, :3])
        odometry = Odometry()
        odometry.header.stamp = source.header.stamp
        odometry.header.frame_id = self._map_frame
        odometry.child_frame_id = self._sensor_frame
        odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z = pose[:3, 3]
        odometry.pose.pose.orientation.x = float(quaternion[0])
        odometry.pose.pose.orientation.y = float(quaternion[1])
        odometry.pose.pose.orientation.z = float(quaternion[2])
        odometry.pose.pose.orientation.w = float(quaternion[3])
        odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.linear.z = twist.linear
        odometry.twist.twist.angular.x, odometry.twist.twist.angular.y, odometry.twist.twist.angular.z = twist.angular
        self._odom_pub.publish(odometry)

        pose_message = PoseStamped()
        pose_message.header = odometry.header
        pose_message.pose = odometry.pose.pose
        self._path.header.stamp = source.header.stamp
        self._path.poses.append(pose_message)
        self._path_pub.publish(self._path)

    def _publish_map(self, source: PointCloud2, points) -> None:
        if points is None or len(points) == 0:
            return
        header = Header(stamp=source.header.stamp, frame_id=self._map_frame)
        self._map_pub.publish(point_cloud2.create_cloud_xyz32(header, points[:, :3]))

    def _save_results(self, _request, response):
        return self._service_call(self._engine.save_results, response)

    def _save_mesh(self, _request, response):
        return self._service_call(self._engine.save_mesh, response)

    @staticmethod
    def _service_call(callback, response):
        try:
            callback()
            response.success = True
            response.message = "saved"
        except Exception as error:
            response.success = False
            response.message = repr(error)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PinSlamNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
