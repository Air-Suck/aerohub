#!/usr/bin/env python3
"""Command and measure the seven-UAV forest traversal."""

import json
import math
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand

from forest_layout import PILLARS, RINGS, RING_TUBE_RADIUS, ring_centerline


INITIAL = [
    (-26.0, 0.0), (-24.2679, -1.0), (-26.0, -2.0),
    (-27.7321, -1.0), (-27.7321, 1.0), (-26.0, 2.0), (-24.2679, 1.0),
]
OFFSETS = [(x + 26.0, y) for x, y in INITIAL]
WAYPOINTS = [(24.0, 0.0)]
FORMATION_SCALE = 1.0
RING_SEGMENTS = []
for ring in RINGS:
    ring_points = ring_centerline(ring, segments=40)
    RING_SEGMENTS.append([
        (start, ring_points[(index + 1) % len(ring_points)])
        for index, start in enumerate(ring_points)
    ])


class Mission:
    def __init__(self):
        self.positions = [None] * 7
        self.commands = [None] * 7
        for index in range(7):
            rospy.Subscriber(f"/drone_{index}_visual_slam/odom", Odometry,
                             self.callback, callback_args=index, queue_size=5)
            rospy.Subscriber(f"/drone_{index}_planning/pos_cmd", PositionCommand,
                             self.command_callback, callback_args=index, queue_size=5)
        self.trigger = rospy.Publisher("/traj_start_trigger", PoseStamped,
                                       queue_size=1, latch=True)

    def callback(self, message, index):
        p = message.pose.pose.position
        self.positions[index] = (p.x, p.y, p.z)

    def command_callback(self, message, index):
        p = message.position
        self.commands[index] = (p.x, p.y, p.z)

    def ready(self):
        return all(position is not None and position[2] > 0.3 for position in self.positions)

    def publish_trigger(self):
        message = PoseStamped()
        message.header.frame_id = "world"
        message.header.stamp = rospy.Time.now()
        message.pose.orientation.w = 1.0
        self.trigger.publish(message)

    def metrics(self):
        center_x = sum(p[0] for p in self.positions) / 7.0
        center_y = sum(p[1] for p in self.positions) / 7.0
        formation_rms = math.sqrt(sum(
            ((p[0] - center_x) - offset[0]) ** 2 +
            ((p[1] - center_y) - offset[1]) ** 2
            for p, offset in zip(self.positions, OFFSETS)
        ) / 7.0)
        min_vehicle_separation, separation_pair = closest_vehicle_pair(self.positions)
        min_obstacle_clearance = float("inf")
        obstacle_detail = None
        for vehicle, position in enumerate(self.positions):
            clearance, kind, obstacle = closest_obstacle(position)
            if clearance < min_obstacle_clearance:
                min_obstacle_clearance = clearance
                obstacle_detail = (vehicle, kind, obstacle)

        tracking_errors = [
            math.dist(position, command) if command is not None else None
            for position, command in zip(self.positions, self.commands)
        ]
        command_separation = float("inf")
        command_clearance = float("inf")
        if all(command is not None for command in self.commands):
            command_separation, _ = closest_vehicle_pair(self.commands)
            command_clearance = min(closest_obstacle(command)[0]
                                    for command in self.commands)
        return (center_x, center_y, formation_rms, min_vehicle_separation,
                min_obstacle_clearance, min(p[2] for p in self.positions),
                separation_pair, obstacle_detail, tracking_errors,
                command_separation, command_clearance)


def point_segment_distance(point, start, end):
    delta = tuple(b - a for a, b in zip(start, end))
    length_squared = sum(value * value for value in delta)
    ratio = sum((p - a) * d for p, a, d in zip(point, start, delta)) / length_squared
    ratio = min(max(ratio, 0.0), 1.0)
    closest = tuple(a + ratio * d for a, d in zip(start, delta))
    return math.dist(point, closest)


def cylinder_clearance(point, pillar):
    x, y, radius, height = pillar
    radial = math.hypot(point[0] - x, point[1] - y) - radius
    vertical = max(-point[2], point[2] - height)
    if radial <= 0.0 and vertical <= 0.0:
        return max(radial, vertical)
    return math.hypot(max(radial, 0.0), max(vertical, 0.0))


def closest_vehicle_pair(positions):
    return min(
        (math.dist(positions[i], positions[j]), (i, j))
        for i in range(7) for j in range(i + 1, 7)
    )


def closest_obstacle(point):
    best = (float("inf"), None, None)
    for index, pillar in enumerate(PILLARS):
        candidate = (cylinder_clearance(point, pillar), "pillar", index)
        if candidate[0] < best[0]:
            best = candidate
    for ring_index, segments in enumerate(RING_SEGMENTS):
        for start, end in segments:
            candidate = (
                point_segment_distance(point, start, end) - RING_TUBE_RADIUS,
                "ring", ring_index,
            )
            if candidate[0] < best[0]:
                best = candidate
    return best


def main():
    rospy.init_node("swarm_forest_mission")
    mission = Mission()
    deadline = time.monotonic() + 180.0
    rate = rospy.Rate(5)
    while not rospy.is_shutdown() and not mission.ready() and time.monotonic() < deadline:
        rate.sleep()
    if not mission.ready():
        raise RuntimeError("seven-vehicle odometry did not become ready")

    max_formation_rms = 0.0
    min_vehicle_separation = float("inf")
    min_vehicle_separation_event = None
    min_obstacle_clearance = float("inf")
    min_obstacle_clearance_event = None
    min_command_vehicle_separation = float("inf")
    min_command_obstacle_clearance = float("inf")
    min_command_altitude = float("inf")
    max_tracking_error = 0.0
    max_tracking_error_by_vehicle = [0.0] * 7
    min_altitude = float("inf")
    min_altitude_by_vehicle = [float("inf")] * 7
    max_local_deviation_by_vehicle = [0.0] * 7
    max_center_lateral_deviation = 0.0
    max_vehicle_cross_track_error = 0.0
    max_local_avoidance_deviation = 0.0
    min_center_y = float("inf")
    max_center_y = float("-inf")
    samples = 0
    center_trace = []
    vehicle_traces = []
    pillars_inside_formation = set()
    final = None
    for _ in range(20):
        mission.publish_trigger()
        rospy.sleep(0.1)

    deadline = time.monotonic() + 120.0
    while (not rospy.is_shutdown() and time.monotonic() < deadline and
           not all(position[2] > 0.55 for position in mission.positions)):
        rate.sleep()
    if not all(position[2] > 0.55 for position in mission.positions):
        raise RuntimeError("seven vehicles did not reach traversal altitude")

    goal_x, goal_y = WAYPOINTS[-1]
    complete = False
    deadline = time.monotonic() + 1200.0
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        snapshot = mission.metrics()
        center_x, center_y, rms, separation, clearance, altitude = snapshot[:6]
        samples += 1
        vehicle_traces.append([
            time.time(), [list(position) for position in mission.positions]
        ])
        if samples % 5 == 0:
            center_trace.append([center_x, center_y])
        for index, (pillar_x, pillar_y, _, _) in enumerate(PILLARS):
            local_x = abs(pillar_x - center_x)
            local_y = abs(pillar_y - center_y)
            full_half_width = math.sqrt(3.0) * FORMATION_SCALE
            half_width = (full_half_width if local_y <= FORMATION_SCALE else
                          full_half_width * max(0.0, (2.0 * FORMATION_SCALE - local_y) / FORMATION_SCALE))
            if local_y <= 2.0 * FORMATION_SCALE and local_x <= half_width:
                pillars_inside_formation.add(index)
        max_formation_rms = max(max_formation_rms, rms)
        if separation < min_vehicle_separation:
            min_vehicle_separation = separation
            min_vehicle_separation_event = {
                "sample": samples,
                "vehicles": list(snapshot[6]),
            }
        if clearance < min_obstacle_clearance:
            min_obstacle_clearance = clearance
            vehicle, kind, obstacle = snapshot[7]
            min_obstacle_clearance_event = {
                "sample": samples,
                "vehicle": vehicle,
                "kind": kind,
                "obstacle": obstacle,
            }
        for index, error in enumerate(snapshot[8]):
            if error is not None:
                max_tracking_error = max(max_tracking_error, error)
                max_tracking_error_by_vehicle[index] = max(
                    max_tracking_error_by_vehicle[index], error)
        min_command_vehicle_separation = min(
            min_command_vehicle_separation, snapshot[9])
        min_command_obstacle_clearance = min(
            min_command_obstacle_clearance, snapshot[10])
        if all(command is not None for command in mission.commands):
            min_command_altitude = min(
                min_command_altitude, min(command[2] for command in mission.commands))
        min_altitude = min(min_altitude, altitude)
        for index, (position, offset) in enumerate(zip(mission.positions, OFFSETS)):
            min_altitude_by_vehicle[index] = min(min_altitude_by_vehicle[index], position[2])
            max_local_deviation_by_vehicle[index] = max(
                max_local_deviation_by_vehicle[index],
                abs((position[1] - center_y) - offset[1]),
            )
        max_center_lateral_deviation = max(max_center_lateral_deviation, abs(center_y))
        max_vehicle_cross_track_error = max(max_vehicle_cross_track_error, max(
            abs(position[1] - offset[1])
            for position, offset in zip(mission.positions, OFFSETS)
        ))
        max_local_avoidance_deviation = max(max_local_avoidance_deviation, max(
            abs((position[1] - center_y) - offset[1])
            for position, offset in zip(mission.positions, OFFSETS)
        ))
        min_center_y = min(min_center_y, center_y)
        max_center_y = max(max_center_y, center_y)
        final = (center_x, center_y, rms, separation, clearance, altitude)
        if math.hypot(center_x - goal_x, center_y - goal_y) < 0.8:
            complete = True
            break
        rate.sleep()

    if complete:
        for _ in range(75):
            rate.sleep()
            vehicle_traces.append([
                time.time(), [list(position) for position in mission.positions]
            ])
        final = mission.metrics()
        max_formation_rms = max(max_formation_rms, final[2])
        min_vehicle_separation = min(min_vehicle_separation, final[3])
        min_obstacle_clearance = min(min_obstacle_clearance, final[4])
        min_altitude = min(min_altitude, final[5])
        min_command_vehicle_separation = min(
            min_command_vehicle_separation, final[9])
        min_command_obstacle_clearance = min(
            min_command_obstacle_clearance, final[10])
        if all(command is not None for command in mission.commands):
            min_command_altitude = min(
                min_command_altitude, min(command[2] for command in mission.commands))
        for index, error in enumerate(final[8]):
            if error is not None:
                max_tracking_error = max(max_tracking_error, error)
                max_tracking_error_by_vehicle[index] = max(
                    max_tracking_error_by_vehicle[index], error)
        for index, position in enumerate(mission.positions):
            min_altitude_by_vehicle[index] = min(min_altitude_by_vehicle[index], position[2])

    if final is None:
        raise RuntimeError("no flight samples")
    result = {
        "complete": complete,
        "configured_waypoints": WAYPOINTS,
        "samples": samples,
        "center_trace": center_trace,
        "vehicle_traces": vehicle_traces,
        "final_center_x_m": final[0],
        "final_center_y_m": final[1],
        "final_formation_rms_m": final[2],
        "max_formation_rms_m": max_formation_rms,
        "min_vehicle_separation_m": min_vehicle_separation,
        "min_vehicle_separation_event": min_vehicle_separation_event,
        "min_obstacle_clearance_m": min_obstacle_clearance,
        "min_obstacle_clearance_event": min_obstacle_clearance_event,
        "min_command_vehicle_separation_m": min_command_vehicle_separation,
        "min_command_obstacle_clearance_m": min_command_obstacle_clearance,
        "min_command_altitude_m": min_command_altitude,
        "max_tracking_error_m": max_tracking_error,
        "max_tracking_error_by_vehicle_m": max_tracking_error_by_vehicle,
        "min_altitude_m": min_altitude,
        "min_altitude_by_vehicle_m": min_altitude_by_vehicle,
        "max_local_deviation_by_vehicle_m": max_local_deviation_by_vehicle,
        "max_center_lateral_deviation_m": max_center_lateral_deviation,
        "max_vehicle_cross_track_error_m": max_vehicle_cross_track_error,
        "max_local_avoidance_deviation_m": max_local_avoidance_deviation,
        "min_center_y_m": min_center_y,
        "max_center_y_m": max_center_y,
        "pillars_inside_formation_count": len(pillars_inside_formation),
        "pillar_count": len(PILLARS),
        "ring_count": len(RINGS),
        "blocked_nominal_path_tree_count": sum(
            -26.0 < pillar[0] < goal_x and
            min(abs(pillar[1] - offset[1]) for offset in OFFSETS) < pillar[2]
            for pillar in PILLARS
        ),
        "nominal_path_min_clearance_m": min(
            abs(pillar[1] - offset[1]) - pillar[2]
            for pillar in PILLARS for offset in OFFSETS
        ),
        "positions": mission.positions,
    }
    with open("/demo/swarm_forest_metrics.json", "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps({key: value for key, value in result.items()
                      if key != "vehicle_traces"}, indent=2))
    if not complete:
        raise RuntimeError("formation did not traverse the forest")


if __name__ == "__main__":
    main()
