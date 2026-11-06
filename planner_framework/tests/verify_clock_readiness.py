"""ROS integration regression: reject START before first /clock without starting a task."""
import time
import tempfile
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from uav_planner_interfaces.srv import ControlPlanner
from uav_planner_framework.ros_node import PlannerFrameworkNode
from uav_planner_framework import BehaviorStatus

root = Path(__file__).resolve().parents[1]
rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', 'registry_root:='+str(root/'registry')])
node = PlannerFrameworkNode()
publisher_node = Node('clock_regression_publisher', use_global_arguments=False,
                      parameter_overrides=[Parameter('use_sim_time', value=False)])
publisher = publisher_node.create_publisher(Clock, '/clock', 10)
request = ControlPlanner.Request(command=ControlPlanner.Request.START,
                                task_config_path='/does/not/exist', deployment_config_path='/does/not/exist')
try:
    response = node._on_control(request, ControlPlanner.Response())
    assert not response.accepted and response.message == 'CLOCK_NOT_READY', response.message
    assert node.lifecycle.snapshot is None
    deadline = time.monotonic()+10
    while node._now() <= 0 and time.monotonic() < deadline:
        message = Clock()
        message.clock.sec = 123
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=.05)
    assert node._now() == 123.0, node._now()
    node._require_clock()
    response = node._on_control(request, ControlPlanner.Response())
    assert not response.accepted and response.message != 'CLOCK_NOT_READY', response.message
    print('PASS: START rejected before /clock without lifecycle mutation; first sample releases clock guard')
    with tempfile.TemporaryDirectory() as resolved:
        node.set_parameters([Parameter('runtime_startup_timeout', value=30.0), Parameter('resolved_dir', value=resolved)])
        node._start_runtime = lambda path: None
        request.task_config_path = str(root/'examples/cat_ora_single.yaml')
        request.deployment_config_path = str(root/'examples/deployment_single.yaml')
        response = node._on_control(request, ControlPlanner.Response())
        assert response.accepted, response.message
        for seconds in (129, 154):
            deadline = time.monotonic()+5
            while node._now() != float(seconds) and time.monotonic() < deadline:
                message = Clock()
                message.clock.sec = seconds
                publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=.05)
            assert node._now() == float(seconds)
            node._tick()
            if seconds == 129:
                assert node.lifecycle.snapshot.status == BehaviorStatus.RUNNING
            else:
                assert node.lifecycle.snapshot.status == BehaviorStatus.FAILED
                assert node.lifecycle.snapshot.failure_reason == 'TRAJECTORY_TIMEOUT'
        print('PASS: separate startup budget permits six seconds but still rejects missing trajectory after thirty seconds')
finally:
    node.destroy_node()
    publisher_node.destroy_node()
    rclpy.shutdown()
