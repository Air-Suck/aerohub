"""Single-aircraft AS2 takeoff → SUPER → hover → land mission."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from as2_msgs.action import Takeoff, Land
from as2_msgs.msg import ControlMode, PlatformInfo, ControllerInfo, PlatformStatus
from as2_msgs.srv import SetControlMode
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool


class SuperMission(Node):
    def __init__(self):
        super().__init__('super_as2_mission')
        for name, value in [('goal', [2.0, 0.0, 1.5]), ('frame_id', 'drone0/odom'),
                            ('goal_topic', '/goal_pose'), ('takeoff_height', 1.5),
                            ('arrival_tolerance', 0.3), ('timeout', 90.0)]:
            self.declare_parameter(name, value)
        self.pose = None
        self.platform = None
        self.controller = None
        self.create_subscription(ControllerInfo, "controller/info",
                                 lambda msg: setattr(self, "controller", msg), qos_profile_sensor_data)
        self.create_subscription(Odometry, 'sensor_measurements/odom', self.on_odometry,
                                 qos_profile_sensor_data)
        self.create_subscription(PlatformInfo, 'platform/info',
                                 lambda msg: setattr(self, 'platform', msg), qos_profile_sensor_data)
        self.goal_pub = self.create_publisher(PoseStamped, self.get_parameter('goal_topic').value, 1)
        self._mission_clients = []

    def on_odometry(self, message):
        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose
        self.pose = pose

    def wait(self, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError('Mission operation timed out')
            rclpy.spin_once(self, timeout_sec=0.02)

    def service(self, kind, name, request):
        client = self.create_client(kind, name)
        self._mission_clients.append(client)
        self.wait(client.service_is_ready)
        future = client.call_async(request)
        self.wait(future.done)
        result = future.result()
        if not result.success:
            raise RuntimeError(f'{name} rejected request')

    def mode(self, mode):
        request = SetControlMode.Request()
        request.control_mode.control_mode = mode
        request.control_mode.yaw_mode = ControlMode.YAW_ANGLE
        self.service(SetControlMode, 'controller/set_control_mode', request)

    def action(self, kind, name, goal, success):
        client = ActionClient(self, kind, name)
        self._mission_clients.append(client)
        self.wait(client.server_is_ready)
        future = client.send_goal_async(goal)
        self.wait(future.done)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError(f'{name} rejected goal')
        result = handle.get_result_async()
        try:
            self.wait(result.done, self.get_parameter('timeout').value)
        except BaseException:
            handle.cancel_goal_async()
            raise
        if not getattr(result.result().result, success):
            raise RuntimeError(f'{name} failed')

    def run(self):
        target = self.get_parameter('goal').value
        if len(target) != 3 or not all(math.isfinite(v) for v in target):
            raise ValueError('goal must contain three finite local ENU coordinates')
        self.wait(lambda: self.pose is not None and self.platform is not None)
        if self.pose.header.frame_id != self.get_parameter('frame_id').value:
            raise ValueError('Local pose does not match configured odom frame')
        # Mission owns reference sources: SUPER adapter must start disabled.
        self.service(SetBool, 'super_adapter/enable', SetBool.Request(data=False))
        self.get_logger().info('Starting AS2 takeoff')
        if not self.platform.armed:
            self.service(SetBool, 'set_arming_state', SetBool.Request(data=True))
        self.wait(lambda: self.platform.armed)
        if not self.platform.offboard:
            self.service(SetBool, 'set_offboard_mode', SetBool.Request(data=True))
        self.wait(lambda: self.platform.offboard)
        self.action(Takeoff, 'TakeoffBehavior',
                    Takeoff.Goal(takeoff_height=self.get_parameter('takeoff_height').value,
                                 takeoff_speed=0.5), 'takeoff_success')
        self.wait(lambda: self.platform.status.state == PlatformStatus.FLYING)
        try:
            self.get_logger().info("Establishing trajectory mode")
            self.mode(ControlMode.TRAJECTORY)
            # Adapter observes controller/info; bounded retries wait for that observation.
            client = self.create_client(SetBool, 'super_adapter/enable')
            self._mission_clients.append(client)
            self.wait(client.service_is_ready)
            deadline = time.monotonic() + 0.35
            while True:
                future = client.call_async(SetBool.Request(data=True))
                self.wait(future.done, 0.3)
                if future.result().success:
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError('SUPER adapter takeover failed')
                rclpy.spin_once(self, timeout_sec=0.02)
            self.wait(lambda: self.goal_pub.get_subscription_count() > 0, 0.1)
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = self.get_parameter('frame_id').value
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = target
            goal.pose.orientation = self.pose.pose.orientation
            self.goal_pub.publish(goal)
            self.get_logger().info("SUPER goal submitted")
            stable_since = None
            def arrived():
                nonlocal stable_since
                if self.controller is not None and self.controller.input_control_mode.control_mode != ControlMode.TRAJECTORY:
                    raise RuntimeError("Trajectory mode lost during SUPER execution")
                p = self.pose.pose.position
                if math.dist((p.x, p.y, p.z), target) > self.get_parameter('arrival_tolerance').value:
                    stable_since = None
                elif stable_since is None:
                    stable_since = time.monotonic()
                return stable_since is not None and time.monotonic() - stable_since >= 1.0
            self.wait(arrived, self.get_parameter('timeout').value)
        finally:
            # Mode switch stops old trajectory even if disabling the adapter fails.
            self.get_logger().info("Switching to hover")
            self.mode(ControlMode.HOVER)
            self.service(SetBool, 'super_adapter/enable', SetBool.Request(data=False))
        self.get_logger().info('Starting AS2 landing')
        self.action(Land, 'LandBehavior', Land.Goal(land_speed=0.5), 'land_success')
        self.wait(lambda: not self.platform.armed, 15.0)
        self.get_logger().info('SUPER mission landed and disarmed')


def main(args=None):
    rclpy.init(args=args)
    node = SuperMission()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
