#!/usr/bin/env python3
"""Bridge AS2 fleet state to Swarm-Formation and sample its native PolyTraj."""

import json
import math
import socketserver
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from traj_utils.msg import PolyTraj


VEHICLES = ["drone%d" % index for index in range(7)]
lock = threading.Lock()
update_lock = threading.Lock()
latest = {}
odom_publishers = {}
goal_publisher = None
broadcast_publisher = None
last_task_id = None
map_health = {}
finished = set()


def trajectory_callback(message):
    with lock:
        latest["drone%d" % message.drone_id] = message


def finish_callback(message, vehicle):
    with lock:
        if message.data:
            finished.add(vehicle)
        else:
            finished.discard(vehicle)


def broadcast_callback(message):
    broadcast_publisher.publish(message)


def evaluate(coefficients, piece, order, value, derivative=0):
    offset = piece * (order + 1)
    return sum(
        coefficients[offset + order - power] * math.factorial(power) / math.factorial(power-derivative) * value ** (power-derivative)
        for power in range(derivative, order + 1)
    )


def sample(message, absolute_time, sample_dt, horizon, derivative=0):
    relative = max(0.0, (absolute_time - Time.from_msg(message.start_time)).nanoseconds / 1e9)
    points = []
    count = max(2, int(round(horizon / sample_dt)) + 1)
    for index in range(count):
        remaining = relative + index * sample_dt
        piece = 0
        while piece + 1 < len(message.duration) and remaining > message.duration[piece]:
            remaining -= message.duration[piece]
            piece += 1
        if derivative and remaining > message.duration[piece]:
            points.append([0.0, 0.0, 0.0])
            continue
        remaining = min(remaining, message.duration[piece])
        points.append([
            evaluate(message.coef_x, piece, message.order, remaining, derivative),
            evaluate(message.coef_y, piece, message.order, remaining, derivative),
            evaluate(message.coef_z, piece, message.order, remaining, derivative),
        ])
    return points


def update(payload):
    global last_task_id
    vehicles = payload["vehicles"]
    if [item["id"] for item in vehicles] != VEHICLES:
        raise ValueError("vehicle IDs must match configured vehicle_count")
    for item in vehicles:
        position = item["position_enu"]
        if len(position) != 3 or not all(math.isfinite(float(v)) for v in position):
            raise ValueError("positions must contain three finite coordinates")
    goal = payload["goal_center_enu"]
    if len(goal) != 3 or not all(math.isfinite(float(v)) for v in goal):
        raise ValueError("goal must contain three finite coordinates")
    for name in ("lead", "sample_dt", "horizon"):
        value = float(payload[name])
        if not math.isfinite(value) or value < 0 or (name != "lead" and value == 0):
            raise ValueError("invalid sampling parameter: " + name)
    if not node.get_parameter("publish_odometry").value:
        # Native live maps report the freshness of both real inputs. Do not
        # resample old polynomial output after a sensor or planner has stopped.
        with lock:
            healthy = all(vehicle in map_health and map_health[vehicle][0] and
                          time.monotonic()-map_health[vehicle][1] < 1.0 for vehicle in VEHICLES)
        if not healthy:
            return {"ok": False, "reason": "WAITING_FOR_TRAJECTORIES",
                    "detail": "waiting for fresh real cloud and odometry from every planner"}

    stamp = node.get_clock().now().to_msg()
    for item in vehicles:
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "world"
        message.child_frame_id = item["id"] + "/base_link"
        position = item["position_enu"]
        message.pose.pose.position.x = float(position[0])
        message.pose.pose.position.y = float(position[1])
        message.pose.pose.position.z = float(position[2])
        message.pose.pose.orientation.w = 1.0
        if item["id"] in odom_publishers:
            odom_publishers[item["id"]].publish(message)

    if payload["task_id"] != last_task_id:
        # Wait for all reliable/durable goal readers before sending a one-shot task.
        if goal_publisher.get_subscription_count() < len(VEHICLES):
            return {"ok": False, "reason": "WAITING_FOR_TRAJECTORIES"}
        with lock:
            latest.clear()
            finished.clear()
        goal = PoseStamped()
        goal.header.stamp = stamp
        goal.header.frame_id = node.get_parameter("map_frame").value
        goal.pose.position.x = float(payload["goal_center_enu"][0])
        goal.pose.position.y = float(payload["goal_center_enu"][1])
        goal.pose.position.z = float(payload["goal_center_enu"][2])
        goal.pose.orientation.w = 1.0
        goal_publisher.publish(goal)
        last_task_id = payload["task_id"]

    with lock:
        messages = [latest.get(vehicle) for vehicle in VEHICLES]
        all_finished = all(vehicle in finished for vehicle in VEHICLES)
    if any(message is None for message in messages):
        return {"ok": False, "reason": "WAITING_FOR_TRAJECTORIES"}

    lead = float(payload["lead"])
    valid_from = node.get_clock().now() + Duration(seconds=lead)
    def sampled(derivative):
        return [sample(message, valid_from, float(payload["sample_dt"]), float(payload["horizon"]), derivative) for message in messages]
    return {
        "ok": True,
        "valid_from": valid_from.nanoseconds / 1e9,
        "trajectories_enu": sampled(0),
        "velocities_enu": sampled(1),
        "accelerations_enu": sampled(2),
        "is_final": all_finished,
    }


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        try:
            payload = json.loads(self.rfile.readline().decode("utf-8"))
            with update_lock:
                result = update(payload)
        except Exception as exc:
            result = {"ok": False, "reason": "ERROR", "error": str(exc)}
        self.wfile.write((json.dumps(result, separators=(",", ":")) + "\n").encode("utf-8"))


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


node = None


def main():
    global node, broadcast_publisher, goal_publisher, VEHICLES
    rclpy.init()
    node = Node("swarm_formation_proxy")
    count = node.declare_parameter("vehicle_count", 7).value
    if not 3 <= count <= 7: raise ValueError("vehicle_count must be in [3,7]")
    VEHICLES = [f"drone{i}" for i in range(count)]
    relay_broadcast = node.declare_parameter("relay_broadcast", True).value
    publish_odometry = node.declare_parameter("publish_odometry", True).value
    node.declare_parameter("map_frame", "world")
    subscriptions = []
    for index, vehicle in enumerate(VEHICLES):
        if publish_odometry:
            odom_publishers[vehicle] = node.create_publisher(
                Odometry, f"/drone_{index}_visual_slam/odom", 20)
        subscriptions.append(node.create_subscription(
            PolyTraj, f"/drone_{index}_planning/trajectory", trajectory_callback, 10))
        subscriptions.append(node.create_subscription(
            Bool, f"/drone_{index}_planning/finish",
            lambda message, vehicle_id=vehicle: finish_callback(message, vehicle_id), 10))
        if not publish_odometry:
            def receive_health(message, vehicle_id=vehicle):
                with lock:
                    map_health[vehicle_id]=(message.data,time.monotonic())
            subscriptions.append(node.create_subscription(
                Bool, f"/drone_{index}_ego_planner_node/grid_map/input_healthy", receive_health,10))
    goal_publisher = node.create_publisher(
        PoseStamped, "/traj_start_trigger",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    broadcast_publisher = node.create_publisher(PolyTraj, "/broadcast_traj_to_planner", 100)
    if relay_broadcast:
        subscriptions.append(node.create_subscription(
            PolyTraj, "/broadcast_traj_from_planner", broadcast_callback, 100))
    executor = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor.start()
    try:
        bind_host = str(node.declare_parameter("bind_host", "127.0.0.1").value)
        with Server((bind_host, 28501), Handler) as server:
            node.get_logger().info(f"Swarm-Formation proxy listening on {bind_host}:28501")
            server.serve_forever()
    finally:
        rclpy.shutdown()
        executor.join(timeout=5.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
