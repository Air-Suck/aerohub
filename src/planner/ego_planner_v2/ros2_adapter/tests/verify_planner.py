#!/usr/bin/env python3
"""Exercise real Humble planner nodes with synthetic ROS inputs, without flight."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import GoalSet
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Empty
from traj_utils.msg import PolyTraj, MINCOTraj


def sample(trajectory, elapsed):
    elapsed = max(0.0, elapsed)
    piece = 0
    while piece < len(trajectory.duration) - 1 and elapsed > trajectory.duration[piece]:
        elapsed -= trajectory.duration[piece]
        piece += 1
    elapsed = min(elapsed, trajectory.duration[piece])
    position, velocity = [], []
    for coefficients in (trajectory.coef_x, trajectory.coef_y, trajectory.coef_z):
        values = coefficients[piece * 6:piece * 6 + 6]
        position.append(sum(value * elapsed ** (5 - j) for j, value in enumerate(values)))
        velocity.append(sum(values[j] * (5 - j) * elapsed ** (4 - j) for j in range(5)))
    return position, velocity


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node("ego_migration_validation")
    processes, streams = [], []
    trajectories, counts, broadcasts, heartbeats = {}, [0, 0], [0, 0], [0, 0]
    goals = [[3.0, 0.0, 2.0], [3.0, 4.0, 2.0]]
    initial = [[0.0, 0.0, 2.0], [0.0, 4.0, 2.0]]
    positions = [list(value) for value in initial]
    velocities = [[0.0] * 3 for _ in range(2)]
    subscriptions = []
    goal_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    goal_pub = node.create_publisher(GoalSet, "/goal_with_id", goal_qos)
    odom_pubs, cloud_pubs, stop_pubs = [], [], []
    errors = []

    def receive(index, message):
        size = len(message.duration)
        values = list(message.duration) + list(message.coef_x) + list(message.coef_y) + list(message.coef_z)
        valid = (message.drone_id == index and message.order == 5 and size > 0
                 and all(len(c) == 6 * size for c in (message.coef_x, message.coef_y, message.coef_z))
                 and all(math.isfinite(value) for value in values)
                 and all(value > 0 for value in message.duration))
        if not valid:
            errors.append(f"Invalid PolyTraj for drone {index}")
            return
        trajectories[index] = message
        counts[index] += 1

    def broadcast(message):
        if message.drone_id in (0, 1):
            broadcasts[message.drone_id] += 1

    subscriptions.append(node.create_subscription(MINCOTraj, "/ego_swarm/trajectory", broadcast, 100))
    report = {"scope": "synthetic ROS input to real Humble planner; sampled feedback, no dynamics"}
    try:
        for index in range(2):
            prefix = f"/drone{index}"
            odom_pubs.append(node.create_publisher(Odometry, prefix + "/navigation/odom", qos_profile_sensor_data))
            cloud_pubs.append(node.create_publisher(PointCloud2, prefix + "/navigation/points", qos_profile_sensor_data))
            stop_pubs.append(node.create_publisher(Empty, prefix + "/ego_planner/mandatory_stop", 1))
            subscriptions.append(node.create_subscription(
                PolyTraj, prefix + "/ego_planner/planning/trajectory",
                lambda message, i=index: receive(i, message), 10))
            def heartbeat(message, i=index):
                heartbeats[i] += 1
            subscriptions.append(node.create_subscription(
                Empty, prefix + "/ego_planner/planning/heartbeat", heartbeat, 10))
            stream = (args.output / f"drone{index}.log").open("w")
            streams.append(stream)
            processes.append(subprocess.Popen([
                "ros2", "launch", "ego_planner", "ego.launch.py", f"drone_id:={index}",
                f"odom_topic:={prefix}/navigation/odom", f"cloud_topic:={prefix}/navigation/points"],
                stdout=stream, stderr=subprocess.STDOUT, start_new_session=True))

        def tick(follow=True):
            now = node.get_clock().now()
            for index in range(2):
                trajectory = trajectories.get(index)
                if follow and trajectory:
                    start = trajectory.start_time.sec + trajectory.start_time.nanosec * 1e-9
                    positions[index], velocities[index] = sample(trajectory, now.nanoseconds * 1e-9 - start)
                odom = Odometry()
                odom.header.stamp = now.to_msg()
                odom.header.frame_id = "world"
                odom.child_frame_id = f"drone{index}/base_link"
                odom.pose.pose.orientation.w = 1.0
                odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = positions[index]
                odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = velocities[index]
                odom_pubs[index].publish(odom)
                cloud = PointCloud2()
                cloud.header = odom.header
                cloud.height, cloud.width = 1, 1
                cloud.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                                for name, offset in (("x", 0), ("y", 4), ("z", 8))]
                cloud.point_step = cloud.row_step = 12
                cloud.is_dense = True
                cloud.data = struct.pack("fff", -5.0, -5.0, 0.0)
                cloud_pubs[index].publish(cloud)
            # Drain both planners' callbacks and bound input publication to 50 Hz.
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.001)
            time.sleep(0.02)
            if any(p.poll() is not None for p in processes):
                raise RuntimeError("Planner process exited unexpectedly")

        deadline = time.monotonic() + 30
        while goal_pub.get_subscription_count() < 2 or min(heartbeats) < 5:
            tick(False)
            if time.monotonic() > deadline:
                raise RuntimeError("Planner discovery/heartbeat timeout")
        # Wrong vehicle goals must not trigger either planner.
        invalid = GoalSet(drone_id=99, goal=[1.0, 1.0, 2.0])
        goal_pub.publish(invalid)
        until = time.monotonic() + 0.5
        while time.monotonic() < until:
            tick(False)
        if any(counts):
            raise RuntimeError("Wrong vehicle goal generated a trajectory")
        for index in range(2):
            goal_pub.publish(GoalSet(drone_id=index, goal=goals[index]))
            until = time.monotonic() + 0.2
            while time.monotonic() < until:
                tick()
        deadline = time.monotonic() + 60
        updated_goal = False
        while time.monotonic() < deadline:
            tick()
            if not updated_goal and min(counts) >= 2:
                goals[0][1] = 0.4
                goals[1][1] = 4.4
                for index in range(2):
                    goal_pub.publish(GoalSet(drone_id=index, goal=goals[index]))
                    until = time.monotonic() + 0.2
                    while time.monotonic() < until:
                        tick()
                updated_goal = True
            distances = [math.dist(positions[i], goals[i]) for i in range(2)]
            if min(counts) >= 2 and max(distances) < 0.15:
                break
        else:
            raise RuntimeError(f"Rolling trajectory/completion timeout, counts={counts}, positions={positions}")
        # Exercise the completion branch for several timer cycles.
        until = time.monotonic() + 2
        while time.monotonic() < until:
            tick()
        if errors or min(broadcasts) == 0:
            raise RuntimeError(f"Message checks failed: {errors}, broadcasts={broadcasts}")
        final_positions = [list(value) for value in positions]
        final_errors = [math.dist(positions[i], goals[i]) for i in range(2)]
        previous_counts = list(counts)
        for publisher in stop_pubs:
            publisher.publish(Empty())
        deadline = time.monotonic() + 5
        while any(counts[i] <= previous_counts[i] for i in range(2)):
            tick(False)
            if time.monotonic() > deadline:
                raise RuntimeError("Native mandatory-stop trajectory timeout")
        for index in range(2):
            endpoint, _ = sample(trajectories[index], sum(trajectories[index].duration))
            if math.dist(endpoint, final_positions[index]) > 0.01:
                raise RuntimeError("Native stop trajectory endpoint is not the current position")
        report.update(passed=True, rolling_goal_update=updated_goal,
                      native_stop_output=True,
                      trajectory_counts=counts, minco_counts=broadcasts,
                      heartbeat_counts=heartbeats, final_positions=final_positions,
                      final_errors=final_errors)
    except Exception as error:
        report.update(passed=False, error=str(error), trajectory_counts=counts,
                      goal_subscribers=goal_pub.get_subscription_count(),
                      odom_subscribers=[publisher.get_subscription_count() for publisher in odom_pubs],
                      discovered_nodes=node.get_node_names_and_namespaces(),
                      minco_counts=broadcasts, heartbeat_counts=heartbeats)
        raise
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
                report["forced_cleanup"] = True
        for stream in streams:
            stream.close()
        (args.output / "result.json").write_text(json.dumps(report, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
