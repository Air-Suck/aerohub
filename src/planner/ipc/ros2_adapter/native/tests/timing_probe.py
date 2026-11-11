"""Synthetic no-flight actual-node clock sensitivity probe; no execution consumer."""
import argparse
import json
import signal
import subprocess
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from uav_follow_reference_acro.msg import ControlSession
from as2_msgs.msg import YawMode

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--clock-hz', type=float, required=True)
parser.add_argument('--duration', type=float, default=6.)
parser.add_argument('--sim-rate', type=float, default=.7)
parser.add_argument('--require-solve-sim-hz', type=float, default=90.)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
if min(args.clock_hz, args.duration, args.sim_rate) <= 0:
    parser.error('rates and duration must be positive')
args.output.mkdir(parents=True, exist_ok=False)
rclpy.init()
node = rclpy.create_node('ipc_clock_sensitivity_probe')
clock = node.create_publisher(Clock, '/clock', 10)
odom = node.create_publisher(Odometry, '/drone0/navigation/odom', qos_profile_sensor_data)
cloud = node.create_publisher(PointCloud2, '/drone0/navigation/points', qos_profile_sensor_data)
session = node.create_publisher(ControlSession, '/drone0/external_acro/session',
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
solves = []
solve_stamps = []
def health(message):
    if message.data.startswith('solved '):
        solves.append(time.monotonic())
        values = dict(item.split('=') for item in message.data.removeprefix('solved ').split(','))
        if 'solve_stamp_ns' in values:
            solve_stamps.append(int(values['solve_stamp_ns']))
node.create_subscription(String, '/drone0/ipc/health', health, 100)
log = (args.output / 'node.log').open('w')
child = subprocess.Popen([get_package_prefix('ipc_native_core') + '/lib/ipc_native_core/ipc_node',
    '--ros-args', '-r', '__ns:=/drone0', '-p', 'use_sim_time:=true'], stdout=log, stderr=subprocess.STDOUT)
try:
    start = time.monotonic()
    next_clock = next_odom = next_cloud = start
    warmup = 2.
    while time.monotonic() < start + warmup + args.duration:
        now = time.monotonic()
        if child.poll() is not None:
            raise RuntimeError('actual IPC node exited')
        logical_ns = int((1000 + (now - start) * args.sim_rate) * 1e9)
        stamp = Clock().clock
        stamp.sec, stamp.nanosec = divmod(logical_ns, 10**9)
        if now >= next_clock:
            clock.publish(Clock(clock=stamp))
            next_clock = now + 1 / args.clock_hz
        if now >= next_odom:
            message = Odometry()
            message.header.stamp = stamp
            message.header.frame_id = 'earth'
            message.child_frame_id = 'drone0/base_link'
            message.pose.pose.position.z = 2.
            message.pose.pose.orientation.w = 1.
            odom.publish(message)
            goal = ControlSession()
            goal.session_id = 'clock-probe'
            goal.revision = 1
            goal.active = True
            goal.target.header.frame_id = 'earth'
            goal.target.point.z = 2.
            goal.yaw.mode = YawMode.FIXED_YAW
            goal.max_speed.x = goal.max_speed.y = goal.max_speed.z = .5
            session.publish(goal)
            next_odom = now + .01
        if now >= next_cloud:
            message = PointCloud2()
            message.header.stamp = stamp
            message.header.frame_id = 'earth'
            message.height = 1
            message.point_step = 12
            message.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                              for i, name in enumerate(('x', 'y', 'z'))]
            cloud.publish(message)
            next_cloud = now + .1
        rclpy.spin_once(node, timeout_sec=.001)
    measured = [t for t in solves if start + warmup <= t <= start + warmup + args.duration]
    if not solve_stamps or len(solve_stamps) != len(solves):
        raise AssertionError('every successful computation must expose its original timestamp')
    if len(set(solve_stamps)) != len(solve_stamps):
        raise AssertionError('successful computations share a source timestamp')
    wall_hz = len(measured) / args.duration
    report = {'scope': 'synthetic stationary inputs, empty map, actual IPC node; no flight',
              'clock_wall_hz_requested': args.clock_hz, 'sim_rate': args.sim_rate,
              'duration_wall_s': args.duration, 'successful_health_records': len(measured),
              'solve_wall_hz': wall_hz, 'solve_sim_hz': wall_hz / args.sim_rate,
              'required_solve_sim_hz': args.require_solve_sim_hz,
              'passed': wall_hz / args.sim_rate >= args.require_solve_sim_hz}
    (args.output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    if not report['passed']:
        raise SystemExit(1)
finally:
    if child.poll() is None:
        child.send_signal(signal.SIGINT)
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=5)
    log.close()
    node.destroy_node()
    rclpy.shutdown()
