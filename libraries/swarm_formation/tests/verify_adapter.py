"""Check that the unchanged adapter consumes the real proxy after runtime replay."""
import math
from pathlib import Path
import tempfile
import time
import yaml

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from uav_planner_interfaces.msg import TimedTrajectorySegment
from uav_swarm_formation_adapter.node import SwarmFormationAdapter, RELATIVE_ENU

vehicles = list(RELATIVE_ENU)
config = {
    "task": {"task_id": "humble-migration-validation"},
    "vehicles": vehicles,
    "planner": {"parameters": {"update_period": .1, "sample_dt": .1, "horizon": 1.0,
                                "valid_from_lead": .2, "max_start_distance": 1.0}},
    "targets": {v: {"x": p[1], "y": -4+p[0], "z": -.5} for v,p in RELATIVE_ENU.items()},
    "deployment": {v: {"px4_origin_in_fleet_ned": [0,0,0],
                       "topics": {"pose": f"/adapter_validation/{v}/pose"}} for v in vehicles},
}
with tempfile.TemporaryDirectory() as directory:
    path = Path(directory)/"resolved.yaml"
    path.write_text(yaml.safe_dump(config))
    rclpy.init(args=["--ros-args", "-p", f"resolved_config:={path}"])
    adapter = SwarmFormationAdapter()
    observer = rclpy.create_node("verify_swarm_adapter")
    received = []
    subscription = observer.create_subscription(
        TimedTrajectorySegment, "/planner/trajectory", received.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publishers = {v: observer.create_publisher(PoseStamped, config["deployment"][v]["topics"]["pose"],10) for v in vehicles}
    executor = SingleThreadedExecutor()
    executor.add_node(adapter)
    executor.add_node(observer)
    try:
        deadline = time.monotonic()+15
        while not received and time.monotonic()<deadline:
            for vehicle,publisher in publishers.items():
                message=PoseStamped()
                relative=RELATIVE_ENU[vehicle]
                message.pose.position.x=-4+2*relative[0]
                message.pose.position.y=2*relative[1]
                message.pose.position.z=.5
                message.pose.orientation.w=1.0
                publisher.publish(message)
            executor.spin_once(timeout_sec=.05)
        assert received, "adapter did not publish a shared trajectory from the native backend"
        message=received[-1]
        assert message.task_id==config["task"]["task_id"] and not message.is_final
        assert [v.vehicle_id for v in message.vehicle_trajectories]==vehicles
        for trajectory in message.vehicle_trajectories:
            assert len(trajectory.points)==11
            assert all(math.isfinite(value) for p in trajectory.points for value in (p.position.x,p.position.y,p.position.z))
        print("PASS unchanged ROS 2 adapter consumed the native backend and published seven shared trajectories")
    finally:
        executor.shutdown()
        adapter.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()
