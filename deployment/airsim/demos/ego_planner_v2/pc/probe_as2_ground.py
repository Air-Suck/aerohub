"""Read-only AS2 ground acceptance: never calls services or sends action goals."""
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from as2_msgs.action import Takeoff, Land, FollowPath
from as2_msgs.msg import PlatformInfo, ControllerInfo, BehaviorStatus, TrajectorySetpoints
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock
from px4_msgs.msg import VehicleControlMode, VehicleCommand
from std_srvs.srv import Trigger, SetBool
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from uav_planner_interfaces.action import FollowTimedPath

rclpy.init()
node = Node('ego_as2_ground_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
rows, latest, subscriptions, clients, actions, failures = {}, {}, [], {}, {}, []
tf_edges = {}
buffer = Buffer()
listener = TransformListener(buffer, node)
started = time.monotonic()


def receive(topic, msg):
    row = rows.setdefault(topic, {'count': 0, 'stamps': set()})
    row['count'] += 1
    row['last_received'] = time.monotonic()
    latest[topic] = msg
    if hasattr(msg, 'header'):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        row['frame'] = msg.header.frame_id
        row['last_stamp_ns'] = stamp
        row['stamps'].add(stamp)
    elif isinstance(msg, Clock):
        row['stamps'].add(msg.clock.sec * 10**9 + msg.clock.nanosec)
    elif isinstance(msg, VehicleControlMode):
        row['stamps'].add(msg.timestamp)
    if isinstance(msg, (PlatformInfo, VehicleControlMode)):
        armed = msg.armed if isinstance(msg, PlatformInfo) else msg.flag_armed
        offboard = msg.offboard if isinstance(msg, PlatformInfo) else msg.flag_control_offboard_enabled
        row.update(armed=armed, offboard=offboard)
        row['ever_armed'] = row.get('ever_armed', False) or armed
        row['ever_offboard'] = row.get('ever_offboard', False) or offboard
    if isinstance(msg, PlatformInfo):
        row['platform_state'] = msg.status.state
    if isinstance(msg, BehaviorStatus):
        row['status'] = msg.status
        row['ever_active'] = row.get('ever_active', False) or msg.status != BehaviorStatus.IDLE
    if isinstance(msg, PointCloud2):
        row['points'] = msg.width * msg.height
        row['ever_empty'] = row.get('ever_empty', False) or row['points'] == 0
    if isinstance(msg, (Odometry, PoseStamped)):
        p = msg.pose.pose if isinstance(msg, Odometry) else msg.pose
        row['position'] = [p.position.x, p.position.y, p.position.z]
        row['quaternion'] = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        if not all(math.isfinite(x) for x in row['position'] + row['quaternion']):
            failures.append('nonfinite pose ' + topic)
    if isinstance(msg, Odometry):
        row['child_frame'] = msg.child_frame_id


def subscribe(topic, kind, qos=qos_profile_sensor_data):
    subscriptions.append(node.create_subscription(kind, topic, lambda m: receive(topic, m), qos))


def observe_tf(msg):
    for transform in msg.transforms:
        child = transform.child_frame_id
        tf_edges.setdefault(child, set()).add(transform.header.frame_id)


subscribe('/clock', Clock)
for topic, qos in [('/tf', qos_profile_sensor_data), ('/tf_static', QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))]:
    subscriptions.append(node.create_subscription(TFMessage, topic, observe_tf, qos))
required = ['/clock']
for vehicle in ('drone0', 'drone1'):
    for suffix, kind in [
        ('platform/info', PlatformInfo), ('controller/info', ControllerInfo),
        ('fmu/out/vehicle_control_mode', VehicleControlMode),
        ('sensor_measurements/odom', Odometry), ('self_localization/pose', PoseStamped),
        ('self_localization/twist', TwistStamped), ('navigation/odom', Odometry),
        ('navigation/points', PointCloud2),
        ('follow_timed_path/_behavior/behavior_status', BehaviorStatus)]:
        topic = '/' + vehicle + '/' + suffix
        required.append(topic)
        subscribe(topic, kind)
    for suffix, kind in [('TakeoffBehavior', Takeoff), ('LandBehavior', Land), ('FollowPathBehavior', FollowPath), ('follow_timed_path', FollowTimedPath)]:
        topic = '/' + vehicle + '/' + suffix
        actions[topic] = ActionClient(node, kind, topic)
    for suffix, kind in [('set_arming_state', SetBool), ('set_offboard_mode', SetBool), ('follow_timed_path/_behavior/stop', Trigger)]:
        topic = '/' + vehicle + '/' + suffix
        clients[topic] = node.create_client(kind, topic)
    subscribe('/' + vehicle + '/fmu/in/vehicle_command', VehicleCommand)
    subscribe('/' + vehicle + '/motion_reference/trajectory', TrajectorySetpoints)

# Fixed startup allowance, followed by 30 seconds of fresh data observation.
deadline = started + 60
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.05)
    if all(t in rows for t in required) and all(a.server_is_ready() for a in actions.values()):
        break
startup_seconds = time.monotonic() - started
for topic in required:
    if topic in rows:
        rows[topic]['count'] = 0
        rows[topic]['stamps'].clear()
measurement_start = time.monotonic()
deadline = measurement_start + 30
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.05)
now = time.monotonic()
ros_now = node.get_clock().now().nanoseconds
connections = {}
for topic in required:
    row = rows.get(topic)
    if row is None:
        failures.append('missing ' + topic)
        continue
    row['last_age_seconds'] = now - row.pop('last_received')
    row['unique_stamps'] = len(row.pop('stamps'))
    if row['count'] < 20 or row['last_age_seconds'] > 1:
        failures.append('stale/low-rate ' + topic)
    if 'last_stamp_ns' in row:
        row['stamp_age_seconds'] = (ros_now - row['last_stamp_ns']) / 1e9
        if not -.1 <= row['stamp_age_seconds'] <= 1 or row['unique_stamps'] < 20:
            failures.append('invalid/nonadvancing time ' + topic)
    if any(row.get(k) for k in ('ever_armed', 'ever_offboard', 'ever_active', 'ever_empty')):
        failures.append('unexpected ground state ' + topic)
    endpoints = node.get_publishers_info_by_topic(topic)
    connections[topic] = {'publishers': [e.node_namespace.rstrip('/') + '/' + e.node_name for e in endpoints],
                          'subscriber_count': node.count_subscribers(topic)}
    if len(endpoints) != 1:
        failures.append('publisher count != 1 ' + topic)
    if '/navigation/' in topic:
        vehicle = topic.split('/')[1]
        if row.get('frame') != vehicle + '/odom':
            failures.append('unexpected navigation frame ' + topic)
        if topic.endswith('/odom') and row.get('child_frame') != vehicle + '/base_link':
            failures.append('unexpected velocity frame ' + topic)
tf_checks = {}
for vehicle in ('drone0', 'drone1'):
    for child in ('base_link', 'Lidar'):
        key = vehicle + '/odom -> ' + vehicle + '/' + child
        try:
            transform = buffer.lookup_transform(vehicle + '/odom', vehicle + '/' + child, Time())
            stamp_ns = transform.header.stamp.sec * 10**9 + transform.header.stamp.nanosec
            tf_checks[key] = {'available': True, 'stamp_sec': stamp_ns / 1e9, 'age_seconds': (ros_now - stamp_ns) / 1e9}
            if not -.1 <= tf_checks[key]['age_seconds'] <= 1:
                failures.append('stale TF ' + key)
        except Exception as exc:
            tf_checks[key] = {'available': False, 'error': str(exc)}
            failures.append('missing TF ' + key)
    for suffix in ('fmu/in/vehicle_command', 'motion_reference/trajectory'):
        if rows.get('/' + vehicle + '/' + suffix, {}).get('count', 0):
            failures.append('unexpected command/reference /' + vehicle + '/' + suffix)
for child, parents in tf_edges.items():
    if len(parents) != 1:
        failures.append('competing TF parents ' + child)
tf_publishers = {topic: [e.node_namespace.rstrip('/') + '/' + e.node_name
                        for e in node.get_publishers_info_by_topic(topic)]
                 for topic in ('/tf', '/tf_static')}
if sorted(tf_publishers['/tf']) != ['/drone0/state_estimator', '/drone1/state_estimator']:
    failures.append('unexpected dynamic TF publishers')
if sorted(tf_publishers['/tf_static']) != ['/drone0/state_estimator', '/drone1/state_estimator', '/platform_sensor_adapter', '/platform_sensor_adapter']:
    failures.append('unexpected static TF publishers')
action_ready = {name: client.server_is_ready() for name, client in actions.items()}
service_ready = {name: client.service_is_ready() for name, client in clients.items()}
failures += ['unavailable action ' + name for name, ready in action_ready.items() if not ready]
failures += ['unavailable service ' + name for name, ready in service_ready.items() if not ready]
for row in rows.values():
    if isinstance(row.get('stamps'), set):
        row['unique_stamps'] = len(row.pop('stamps'))
result = {'passed': not failures, 'failures': sorted(set(failures)), 'startup_seconds': startup_seconds,
          'observation_seconds': now - measurement_start, 'topics': rows, 'connections': connections,
          'actions_ready': action_ready, 'services_ready': service_ready, 'tf_checks': tf_checks,
          'tf_parents': {k: sorted(v) for k, v in tf_edges.items()}, 'tf_publishers': tf_publishers,
          'nodes': sorted(ns.rstrip('/') + '/' + name for name, ns in node.get_node_names_and_namespaces()),
          'scope': 'real AirSim/PX4 to AS2 ground interfaces only; no goals, arming, takeoff or EGO planning',
          'gates': {'startup_allowance_s': 60, 'observation_s': 30, 'minimum_messages': 20,
                    'max_receive_age_s': 1, 'max_stamp_age_s': 1, 'min_stamp_age_s': -.1,
                    'one_publisher_per_required_topic': True, 'expected_tf_publishers_and_unique_parents': True,
                    'disarmed_and_not_offboard': True, 'idle_timed_behavior': True}}
Path('/evidence/result.json').write_text(json.dumps(result, indent=2))
print(json.dumps({'passed': result['passed'], 'failures': result['failures'], 'startup_seconds': startup_seconds}, indent=2))
node.destroy_node()
rclpy.shutdown()
raise SystemExit(bool(failures))
