#!/usr/bin/env python3
"""Bridge AS2 fleet state to Swarm-Formation and sample its native PolyTraj."""

import json
import math
import socketserver
import socket
import struct
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from traj_utils.msg import PolyTraj
from rosgraph_msgs.msg import Clock


VEHICLES = ["drone%d" % index for index in range(7)]
# Upstream uses start_time < 1e9 as an uninitialized sentinel. Preserve that
# contract without running the planner on wall time: epoch + Gazebo elapsed.
ROS1_SIM_EPOCH = 1700000000
lock = threading.Lock()
latest = {}
odom_publishers = {}
goal_publisher = None
broadcast_publisher = None
last_task_id = None


def trajectory_callback(message):
    with lock:
        vehicle = "drone%d" % message.drone_id
        previous = latest.get(vehicle)
        if previous is not None and message.start_time < previous.start_time:
            rospy.logwarn("Ignoring out-of-order native trajectory for %s", vehicle)
            return
        latest[vehicle] = message
    if previous is not None:
        # Diagnose the native handoff separately from adapter/executor updates.
        # Do not repair a discontinuity by silently retiming or blending curves.
        differences = []
        for derivative in range(3):
            old = sample(previous, message.start_time, .1, .1, derivative)[0]
            new = sample(message, message.start_time, .1, .1, derivative)[0]
            differences.append(math.sqrt(sum((a-b)**2 for a, b in zip(old, new))))
        rospy.loginfo("native_handoff vehicle=%s start=%.9f age=%.6f dp=%.6f dv=%.6f da=%.6f",
                      vehicle, message.start_time.to_sec(),
                      (rospy.Time.now()-message.start_time).to_sec(), *differences)


def broadcast_callback(message):
    broadcast_publisher.publish(message)


def evaluate(coefficients, piece, order, value, derivative=0):
    offset = piece * (order + 1)
    return sum(
        coefficients[offset + order - power]
        * (power if derivative >= 1 else 1)
        * (power-1 if derivative >= 2 else 1)
        * value ** (power-derivative)
        for power in range(derivative, order + 1)
    )


def sample(message, absolute_time, sample_dt, horizon, derivative=0):
    relative = (absolute_time - message.start_time).to_sec()
    if relative < 0:
        raise ValueError("trajectory is not valid before its native start_time")
    points = []
    count = max(2, int(round(horizon / sample_dt)) + 1)
    for index in range(count):
        remaining = relative + index * sample_dt
        piece = 0
        while piece + 1 < len(message.duration) and remaining > message.duration[piece]:
            remaining -= message.duration[piece]
            piece += 1
        beyond_end = remaining > message.duration[piece]
        remaining = min(remaining, message.duration[piece])
        points.append([
            evaluate(message.coef_x, piece, message.order, remaining, derivative),
            evaluate(message.coef_y, piece, message.order, remaining, derivative),
            evaluate(message.coef_z, piece, message.order, remaining, derivative),
        ] if not (beyond_end and derivative) else [0.0,0.0,0.0])
    return points


def update(payload):
    global last_task_id
    vehicles = payload["vehicles"]
    if [item["id"] for item in vehicles] != VEHICLES:
        raise ValueError("Swarm-Formation deployment requires drone0..drone6")

    stamp = rospy.Time.now()
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
        odom_publishers[item["id"]].publish(message)

    if payload["task_id"] != last_task_id:
        with lock:
            latest.clear()
        goal = PoseStamped()
        goal.header.stamp = stamp
        goal.header.frame_id = "world"
        goal.pose.position.x = float(payload["goal_center_enu"][0])
        goal.pose.position.y = float(payload["goal_center_enu"][1])
        goal.pose.position.z = float(payload["goal_center_enu"][2])
        goal.pose.orientation.w = 1.0
        goal_publisher.publish(goal)
        last_task_id = payload["task_id"]

    with lock:
        messages = [latest.get(vehicle) for vehicle in VEHICLES]
    if any(message is None for message in messages):
        return {"ok": False, "reason": "WAITING_FOR_TRAJECTORIES"}

    valid_from = rospy.Time.from_sec(ROS1_SIM_EPOCH + float(payload["valid_from"]))
    if valid_from <= rospy.Time.now():
        return {"ok": False, "reason": "LATE_REQUEST"}
    if any(message.start_time > valid_from for message in messages):
        return {"ok": False, "reason": "WAITING_FOR_NATIVE_START"}
    return {
        "ok": True,
        "valid_from": float(payload["valid_from"]),
        "velocities_enu": [sample(message, valid_from, float(payload["sample_dt"]),
            float(payload["horizon"]), 1) for message in messages],
        "accelerations_enu": [sample(message, valid_from, float(payload["sample_dt"]),
            float(payload["horizon"]), 2) for message in messages],
        "trajectories_enu": [
            sample(
                message,
                valid_from,
                float(payload["sample_dt"]),
                float(payload["horizon"]),
            )
            for message in messages
        ],
    }


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        try:
            payload = json.loads(self.rfile.readline().decode("utf-8"))
            result = update(payload)
        except Exception as exc:
            result = {"ok": False, "reason": "ERROR", "error": str(exc)}
        self.wfile.write((json.dumps(result, separators=(",", ":")) + "\n").encode("utf-8"))


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    global broadcast_publisher, goal_publisher
    rospy.init_node("swarm_formation_proxy")
    clock_publisher = rospy.Publisher("/clock", Clock, queue_size=10)
    def forward_clock():
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 28503))
        receiver.settimeout(0.5)
        try:
            while not rospy.is_shutdown():
                try:
                    payload, _ = receiver.recvfrom(12)
                except socket.timeout:
                    continue
                if len(payload) == 12:
                    sec, nsec = struct.unpack("!qI", payload)
                    clock_publisher.publish(Clock(clock=rospy.Time(sec + ROS1_SIM_EPOCH, nsec)))
        finally:
            receiver.close()
    threading.Thread(target=forward_clock, daemon=True).start()
    for index, vehicle in enumerate(VEHICLES):
        odom_publishers[vehicle] = rospy.Publisher(
            "/drone_%d_visual_slam/odom" % index, Odometry, queue_size=20
        )
        rospy.Subscriber(
            "/drone_%d_planning/trajectory" % index,
            PolyTraj,
            trajectory_callback,
            queue_size=10,
        )
    goal_publisher = rospy.Publisher("/traj_start_trigger", PoseStamped, queue_size=1, latch=True)
    broadcast_publisher = rospy.Publisher(
        "/broadcast_traj_to_planner", PolyTraj, queue_size=100
    )
    rospy.Subscriber(
        "/broadcast_traj_from_planner", PolyTraj, broadcast_callback, queue_size=100
    )
    with Server(("127.0.0.1", 28501), Handler) as server:
        rospy.loginfo("Swarm-Formation proxy listening on 127.0.0.1:28501")
        server.serve_forever()


if __name__ == "__main__":
    main()
