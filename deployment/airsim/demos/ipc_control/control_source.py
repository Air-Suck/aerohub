"""Compute-role validation controller, not IPC and not a navigation benchmark."""
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool
from uav_follow_reference_acro.msg import ControlSession, ControlFrame


class Source(Node):
    def __init__(self):
        super().__init__('acro_control_validation_source', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.pose = self.twist = self.session = None
        self.received = {}
        self.sequence = 0
        self.integral = 0.0
        self.start = time.monotonic()
        self.hover = None
        self.pub = self.create_publisher(ControlFrame, '/drone0/external_acro/control',
                                         qos_profile_sensor_data)
        self.ready_pub = self.create_publisher(Bool, '/drone0/acro_validation_source/ready', 1)
        self.create_subscription(ControlSession, '/drone0/external_acro/session', self.on_session,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PoseStamped, '/drone0/self_localization/pose',
            lambda m: self.store('pose', m), qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/drone0/self_localization/twist',
            lambda m: self.store('twist', m), qos_profile_sensor_data)
        self.create_timer(0.01, self.tick)

    def store(self, name, msg):
        setattr(self, name, msg)
        self.received[name] = time.monotonic()

    def on_session(self, msg):
        if self.session is None or (msg.session_id, msg.revision) != (
                self.session.session_id, self.session.revision):
            self.integral = 0.0
        self.session = msg

    def tick(self):
        if self.hover is None and Path('/evidence/hover.json').exists():
            self.hover = json.loads(Path('/evidence/hover.json').read_text())['normalized_thrust']
        ready = bool(self.pose and self.twist and all(
            time.monotonic()-self.received[k] <= .15 for k in ('pose', 'twist')) and self.hover is not None)
        self.ready_pub.publish(Bool(data=ready))
        if not ready or not self.session or not self.session.active:
            return
        if Path('/evidence/stop-source').exists():
            return  # Deliberate input-loss case; recovery is owned by the Behavior.
        q = self.pose.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        p = self.pose.pose.position
        target = self.session.target.point
        error = np.array([target.x-p.x, target.y-p.y, target.z-p.z])
        v = self.twist.twist.linear
        velocity = np.array([v.x, v.y, v.z])
        if self.twist.header.frame_id == 'drone0/base_link':
            velocity = R @ velocity
        elif self.twist.header.frame_id != self.pose.header.frame_id:
            return
        self.integral = float(np.clip(self.integral + error[2]*.01, -.5, .5))
        acceleration = np.clip(1.5*error - 1.5*velocity, -1.0, 1.0)
        acceleration[2] += 0.4*self.integral + 9.81
        zd = acceleration / np.linalg.norm(acceleration)
        yaw = self.session.yaw.angle
        xc = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        yd = np.cross(zd, xc); yd /= np.linalg.norm(yd)
        Rd = np.column_stack((np.cross(yd, zd), yd, zd))
        skew = (Rd.T@R - R.T@Rd)*.5
        rates = np.clip(-4.0*np.array([skew[2, 1], skew[0, 2], skew[1, 0]]), -.5, .5)
        throttle = float(np.clip(self.hover*np.dot(acceleration, R[:, 2])/9.81, .1, .85))
        msg = ControlFrame()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'drone0/base_link'
        msg.last_successful_solve = msg.header.stamp
        msg.solver_ok = True
        msg.session_id = self.session.session_id
        msg.revision = self.session.revision
        self.sequence += 1; msg.sequence = self.sequence
        msg.body_rates.x, msg.body_rates.y, msg.body_rates.z = map(float, rates)
        msg.thrust_newtons = throttle*15.0  # Matches the explicit test platform max_thrust.
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Source()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
