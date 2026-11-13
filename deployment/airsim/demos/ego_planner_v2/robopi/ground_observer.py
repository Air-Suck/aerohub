"""Read-only, bounded ground observer; never creates control publishers or goals."""
import json
import sys
import time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
from interfaces import topic_map

TOPICS = {
    '/clock': 'rosgraph_msgs/msg/Clock',
    '/drone0/platform/info': 'as2_msgs/msg/PlatformInfo',
    '/drone0/fmu/out/vehicle_control_mode': 'px4_msgs/msg/VehicleControlMode',
    '/drone0/sensor_measurements/odom': 'nav_msgs/msg/Odometry',
    '/drone0/sensor_measurements/Lidar/points': 'sensor_msgs/msg/PointCloud2',
    '/drone0/navigation/odom': 'nav_msgs/msg/Odometry',
    '/drone0/navigation/points': 'sensor_msgs/msg/PointCloud2',
    '/drone0/follow_timed_path/_behavior/behavior_status': 'as2_msgs/msg/BehaviorStatus',
    '/drone0/TakeoffBehavior/_behavior/behavior_status': 'as2_msgs/msg/BehaviorStatus',
    '/drone0/LandBehavior/_behavior/behavior_status': 'as2_msgs/msg/BehaviorStatus',
    '/ego/status': 'std_msgs/msg/String',
    '/ego/drone0/odom': 'nav_msgs/msg/Odometry',
    '/ego/drone0/points': 'sensor_msgs/msg/PointCloud2',
    '/drone0/ego_planner/planning/heartbeat': 'std_msgs/msg/Empty',
}


def main():
    output = Path(sys.argv[1])
    duration = float(sys.argv[2]) if len(sys.argv)>2 else 40
    mapping = topic_map(json.loads((output.parent/'config.json').read_text()))
    resolved = {topic: mapping.get(topic, topic) for topic in TOPICS}
    rclpy.init()
    node = rclpy.create_node('ego_single_ground_observer')
    rows = {topic: dict(count=0) for topic in TOPICS}
    def receive(topic, message):
        row = rows[topic]
        now = time.monotonic()
        if row['count']:
            row['max_gap'] = max(row.get('max_gap', 0), now-row['last'])
        row.update(count=row['count']+1, last=now)
        for attr in ('armed', 'flag_armed', 'status', 'data', 'width', 'height'):
            if hasattr(message, attr) and (attr != 'data' or isinstance(message.data, str)):
                value = getattr(message, attr)
                row[attr] = message_to_ordereddict(value) if hasattr(value, 'get_fields_and_field_types') else value
        header = getattr(message, 'header', None)
        stamp = header.stamp if header else getattr(message, 'clock', None)
        if header:
            row['frame'] = header.frame_id
        if stamp:
            row['stamp'] = stamp.sec+stamp.nanosec/1e9
    subs = [node.create_subscription(get_message(kind), resolved[topic],
            lambda m, t=topic:receive(t, m), qos_profile_sensor_data) for topic,kind in TOPICS.items()]
    deadline = time.monotonic()+duration
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
    graph = {}
    for topic in TOPICS:
        graph[topic] = {kind:[dict(node=i.node_name, namespace=i.node_namespace,
            type=i.topic_type, gid=list(i.endpoint_gid)) for i in method(resolved[topic])]
            for kind,method in [('publishers',node.get_publishers_info_by_topic),
                                ('subscriptions',node.get_subscriptions_info_by_topic)]}
        if rows[topic]['count']:
            rows[topic]['receipt_age'] = time.monotonic()-rows[topic]['last']
    result = dict(duration=duration, topics=rows, graph=graph, resolved_topics=resolved,
        all_topics=node.get_topic_names_and_types(), nodes=node.get_node_names_and_namespaces())
    output.write_text(json.dumps(result,indent=2))
    print(json.dumps(rows,indent=2),flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
