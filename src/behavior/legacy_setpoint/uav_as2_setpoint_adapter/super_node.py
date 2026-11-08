"""Thin SUPER PVA adapter. Mode ownership stays with the mission runner."""
import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from as2_msgs.msg import ControlMode, ControllerInfo, TrajectoryPoint, TrajectorySetpoints
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mars_quadrotor_msgs.msg import PositionCommand
from std_srvs.srv import SetBool

from .node import quaternion_yaw


def convert_command(message, frame, held_yaw, follow_yaw=False):
    # SUPER uses 1 for primary and 2 for backup. Zero means no valid sample.
    if message.trajectory_flag not in (1, 2):
        raise ValueError("SUPER has no executable trajectory")
    if message.header.frame_id != frame:
        raise ValueError("SUPER must publish in the vehicle local ENU odom frame")
    result = TrajectorySetpoints()
    result.header = copy.deepcopy(message.header)
    point = TrajectoryPoint()
    for destination, source in ((point.position, message.position),
                                (point.twist, message.velocity),
                                (point.acceleration, message.acceleration)):
        values = (source.x, source.y, source.z)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("SUPER PVA must be finite")
        destination.x, destination.y, destination.z = values
    point.yaw_angle = message.yaw if follow_yaw else held_yaw
    if point.yaw_angle is None or not math.isfinite(point.yaw_angle):
        raise ValueError("No finite takeover yaw available")
    result.setpoints = [point]
    return result


class SuperAdapter(Node):
    def __init__(self):
        super().__init__("super_as2_adapter")
        self.declare_parameter("frame_id", "drone0/odom")
        self.declare_parameter("command_topic", "position_cmd")
        self.declare_parameter("follow_yaw", False)
        self.frame = self.get_parameter("frame_id").value
        self.current_yaw = None
        self.held_yaw = None
        self.ready = False
        self.enabled = False
        self.publisher = self.create_publisher(TrajectorySetpoints, "motion_reference/trajectory", 1)
        self.create_subscription(Odometry, "sensor_measurements/odom", self.on_pose,
                                 qos_profile_sensor_data)
        self.create_subscription(ControllerInfo, "controller/info", self.on_mode,
                                 qos_profile_sensor_data)
        self.create_subscription(PositionCommand, self.get_parameter("command_topic").value,
                                 self.on_command, qos_profile_sensor_data)
        self.create_service(SetBool, "super_adapter/enable", self.on_enable)

    def on_pose(self, message):
        if message.header.frame_id == self.frame:
            self.current_yaw = quaternion_yaw(message.pose.pose.orientation)

    def on_mode(self, message):
        mode = message.input_control_mode
        self.ready = (mode.control_mode == ControlMode.TRAJECTORY and
                      mode.yaw_mode == ControlMode.YAW_ANGLE)
        if not self.ready:
            self.enabled = False

    def on_enable(self, request, response):
        if not request.data:
            self.enabled = False
            response.success = True
        elif self.ready and self.current_yaw is not None and math.isfinite(self.current_yaw):
            self.held_yaw = self.current_yaw
            self.enabled = True
            response.success = True
        else:
            response.success = False
            response.message = "Establish trajectory mode and local pose before enabling"
        return response

    def on_command(self, message):
        if not self.enabled or not self.ready:
            return
        try:
            result = convert_command(message, self.frame, self.held_yaw,
                                     self.get_parameter("follow_yaw").value)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return
        self.publisher.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = SuperAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
