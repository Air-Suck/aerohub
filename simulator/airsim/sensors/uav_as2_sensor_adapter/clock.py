"""One global ROS clock sourced from AirSim's simulation clock."""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from .rpc import Rpc


class AirSimClock(Node):
    def __init__(self):
        super().__init__('platform_clock_bridge')
        for name, value in [('rpc_host', '127.0.0.1'), ('rpc_port', 41471),
                            ('vehicle_name', 'PX4'), ('rate_hz', 50.0)]:
            self.declare_parameter(name, value)
        self.rpc = Rpc(self.get_parameter('rpc_host').value,
                       self.get_parameter('rpc_port').value, timeout=1.0)
        self.vehicle = self.get_parameter('vehicle_name').value
        rate = float(self.get_parameter('rate_hz').value)
        if not 0 < rate <= 1000:
            raise ValueError('rate_hz must be in (0, 1000]')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(Clock, '/clock', qos)
        self.last = None
        self.error_time = -10**30
        # The bridge itself uses wall time; consumers use simulation time.
        if self.get_parameter('use_sim_time').value:
            raise ValueError('clock bridge must run with use_sim_time=false')
        self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        try:
            timestamp = int(self.rpc.call('getMultirotorState', self.vehicle)['timestamp'])
            if timestamp <= 0:
                raise ValueError('invalid AirSim timestamp')
            if self.last is not None and timestamp < self.last:
                self.get_logger().warning('AirSim time moved backwards; consumers must handle reset')
            msg = Clock()
            msg.clock.sec, msg.clock.nanosec = divmod(timestamp, 1_000_000_000)
            self.publisher.publish(msg)
            self.last = timestamp
        except (OSError, RuntimeError, ValueError, KeyError, TypeError) as exc:
            now = self.get_clock().now().nanoseconds
            if now - self.error_time > 5_000_000_000:
                self.get_logger().error(f'AirSim clock unavailable: {exc}')
                self.error_time = now


def main():
    rclpy.init()
    node = AirSimClock()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
