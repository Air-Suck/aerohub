"""Real ROS 2 synthetic-input harness; does not claim platform or flight evidence."""
import json
import struct
import time

import rclpy
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String
from uav_planner_interfaces.msg import TimedPath


def main():
    rclpy.init()
    node = rclpy.create_node('gcopter_contract_check')
    odom_pub = node.create_publisher(Odometry, '/drone0/sensor_measurements/odom', 10)
    map_pub = node.create_publisher(PointCloud2, '/drone0/gcopter/map', 10)
    plans = []
    sub = node.create_subscription(TimedPath, '/drone0/gcopter/plan', plans.append, 10)
    statuses = []
    status_sub = node.create_subscription(String, '/drone0/gcopter/state',
        lambda msg: statuses.append(json.loads(msg.data)), 10)
    enabler = node.create_client(SetBool, '/drone0/gcopter/enable')
    assert enabler.wait_for_service(timeout_sec=10)
    clients = {name: node.create_client(Trigger, '/drone0/gcopter/' + name)
               for name in ('plan_once', 'health', 'reset')}
    for client in clients.values():
        assert client.wait_for_service(timeout_sec=10), 'service discovery'

    def spin_until(predicate, timeout=5):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert predicate(), 'timeout'

    def call(name):
        future = clients[name].call_async(Trigger.Request())
        spin_until(future.done, 15)
        return future.result()

    def inputs(frame='drone0/odom', occupied=False, age=0.0):
        for _ in range(12):
            odom = Odometry()
            odom.header.stamp = (node.get_clock().now()-Duration(seconds=age)).to_msg()
            odom.header.frame_id = frame
            odom.child_frame_id = 'drone0/base_link'
            odom.pose.pose.position.x = -3.0
            odom.pose.pose.position.z = 2.0
            odom.pose.pose.orientation.w = 1.0
            cloud = PointCloud2()
            cloud.header = odom.header
            cloud.height = 1
            cloud.width = 1
            cloud.fields = [PointField(name=n, offset=i*4, datatype=7, count=1)
                            for i, n in enumerate(('x', 'y', 'z'))]
            cloud.point_step = 12
            cloud.row_step = 12
            cloud.data = struct.pack('<fff', -3.0 if occupied else 15.0, 0.0, 2.0)
            cloud.is_dense = True
            odom_pub.publish(odom)
            map_pub.publish(cloud)
            rclpy.spin_once(node, timeout_sec=0.03)

    assert not call('health').success
    assert not call('plan_once').success
    inputs(frame='wrong')
    assert not call('health').success
    inputs(age=5.)
    assert not call('health').success
    inputs(age=-5.)
    assert not call('health').success
    inputs(occupied=True)
    assert call('health').success
    assert call('plan_once').message == 'invalid_endpoint'
    inputs()
    enabled = enabler.call_async(SetBool.Request(data=True))
    spin_until(enabled.done)
    assert enabled.result().success
    response = call('plan_once')
    assert response.success, response.message
    spin_until(lambda: len(plans) == 1)
    plan = plans[0]
    times = [p.time_from_start.sec*10**9+p.time_from_start.nanosec for p in plan.points]
    assert times[0] == 0 and all(b > a for a, b in zip(times, times[1:]))
    assert abs(plan.points[-1].position.x - 3) < 1e-5
    assert plan.frame_id == 'drone0/odom' and plan.valid_fields == 7
    spin_until(lambda: any('PLAN_REJECTED' in s.get('error', '') for s in statuses))
    # With no Behavior host, the real adapter must reject rather than publish references.
    assert any('unavailable' in s.get('error', '') for s in statuses)
    assert call('reset').success
    assert not call('health').success
    assert not call('plan_once').success
    print(json.dumps({'synthetic_ros_checks': 'passed', 'points': len(times),
                      'duration_ns': times[-1], 'platform_input': 'not-tested',
                      'behavior_consumer': 'not-tested'}))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
