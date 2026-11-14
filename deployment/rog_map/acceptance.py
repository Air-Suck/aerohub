#!/usr/bin/env python3
"""Consume ROG-Map through its real ROS 2 interfaces during acceptance."""

import json
import math
import os
from pathlib import Path
import struct
import time

import rclpy
from as2_msgs.action import GoToWaypoint, Land, Takeoff
from as2_msgs.msg import ControlMode, PlatformInfo, PlatformStatus
from as2_msgs.srv import SetControlMode
from geometry_msgs.msg import Point, PoseStamped
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleControlMode, VehicleStatus
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool
from uav_rog_map_interfaces.msg import MapStatus
from uav_rog_map_interfaces.srv import CheckLine, QueryVoxel


class Acceptance:
    def __init__(self):
        self.output = Path(os.environ.get("ROG_MAP_RESULTS", "/results"))
        self.output.mkdir(parents=True, exist_ok=True)
        self.scenario = os.environ.get("ROG_MAP_SCENARIO", "normal")
        if self.scenario not in ("normal", "sensor_fault", "algorithm_fault"):
            raise ValueError("invalid ROG_MAP_SCENARIO")

        self.node = rclpy.create_node("rog_map_acceptance")
        self.latest = {}
        self.counts = {}
        self.received_at = {}
        self.status_history = []
        self.update_stamps = []
        self.poses = []
        self.acks = {}
        self.clients = []
        self.subscriptions = []
        self.occupied_point = None
        self.output_widths = {"occupied": 0, "inflated": 0, "esdf": 0}

        sensor_topics = {
            "/drone0/self_localization/pose": PoseStamped,
            "/drone0/platform/info": PlatformInfo,
            "/drone0/sensor_measurements/Lidar/points": PointCloud2,
            "/drone0/navigation/points": PointCloud2,
            "/drone0/fmu/out/vehicle_control_mode": VehicleControlMode,
            "/drone0/fmu/out/vehicle_status_v4": VehicleStatus,
        }
        for topic, kind in sensor_topics.items():
            callback = lambda message, topic=topic: self.receive(topic, message)
            self.subscriptions.append(self.node.create_subscription(
                kind, topic, callback, qos_profile_sensor_data))

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.subscriptions.append(self.node.create_subscription(
            MapStatus, "/drone0/map/status", self.status, reliable))
        for name, topic in (
                ("occupied", "/drone0/map/occupied"),
                ("inflated", "/drone0/map/inflated_occupied"),
                ("esdf", "/drone0/map/esdf_slice")):
            self.subscriptions.append(self.node.create_subscription(
                PointCloud2, topic,
                lambda message, name=name: self.map_output(name, message), reliable))
        self.subscriptions.append(self.node.create_subscription(
            VehicleCommandAck,
            "/drone0/fmu/out/vehicle_command_ack_v" + str(VehicleCommandAck.MESSAGE_VERSION),
            lambda message: self.acks.__setitem__(message.command, message.result),
            qos_profile_sensor_data))

        self.query = self.node.create_client(QueryVoxel, "/drone0/map/query_voxel")
        self.line = self.node.create_client(CheckLine, "/drone0/map/check_line")
        self.vehicle_command = self.node.create_publisher(
            VehicleCommand, "/drone0/fmu/in/vehicle_command", qos_profile_sensor_data)

    def receive(self, topic, message):
        self.latest[topic] = message
        self.counts[topic] = self.counts.get(topic, 0) + 1
        self.received_at[topic] = time.monotonic()
        if topic == "/drone0/self_localization/pose":
            point = message.pose.position
            if all(math.isfinite(value) for value in (point.x, point.y, point.z)):
                self.poses.append((point.x, point.y, point.z))

    @staticmethod
    def stamp_seconds(stamp):
        return stamp.sec + stamp.nanosec / 1e9

    def status(self, message):
        received = time.monotonic()
        self.latest["status"] = message
        self.received_at["status"] = received
        self.counts["status"] = self.counts.get("status", 0) + 1
        update = self.stamp_seconds(message.last_update)
        if update > 0 and (not self.update_stamps or update != self.update_stamps[-1]):
            self.update_stamps.append(update)
        self.status_history.append({
            "wall": time.time(),
            "ready": bool(message.ready),
            "epoch_valid": bool(message.epoch_valid),
            "accepted_clouds": int(message.accepted_clouds),
            "last_update": update,
            "input_age": float(message.input_age),
        })

    def map_output(self, name, message):
        self.output_widths[name] = max(self.output_widths[name], int(message.width))
        if name != "occupied" or message.width == 0:
            return
        fields = {field.name: field for field in message.fields}
        if not {"x", "y", "z"} <= fields.keys():
            return
        endian = ">" if message.is_bigendian else "<"
        for index in range(message.width * message.height):
            base = index * message.point_step
            values = tuple(struct.unpack_from(
                endian + "f", message.data, base + fields[axis].offset)[0]
                for axis in ("x", "y", "z"))
            if all(math.isfinite(value) for value in values):
                self.occupied_point = values
                return

    def spin_until(self, predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError(label)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def marker(self, name, payload=None):
        data = {"wall_time": time.time()}
        if payload:
            data.update(payload)
        (self.output / name).write_text(json.dumps(data, indent=2) + "\n")

    def wait_file(self, name, timeout=30):
        self.spin_until(lambda: (self.output / name).is_file(), timeout, "missing " + name)
        return json.loads((self.output / name).read_text())

    def call(self, client, request, timeout=5):
        self.spin_until(client.service_is_ready, timeout, "map service unavailable")
        future = client.call_async(request)
        self.spin_until(future.done, timeout, "map service call timed out")
        if future.result() is None:
            raise RuntimeError("map service returned no response")
        return future.result()

    def query_observed_voxel(self):
        if self.occupied_point is None:
            raise RuntimeError("no occupied voxel snapshot was received")
        query = QueryVoxel.Request()
        query.point = Point(
            x=self.occupied_point[0], y=self.occupied_point[1], z=self.occupied_point[2])
        result = self.call(self.query, query)
        if not result.ready or result.occupancy != QueryVoxel.Response.OCCUPIED:
            raise RuntimeError("published occupied voxel was not queryable as occupied")

        line = CheckLine.Request()
        line.start = Point(
            x=self.occupied_point[0] - 0.5,
            y=self.occupied_point[1], z=self.occupied_point[2])
        line.end = Point(
            x=self.occupied_point[0] + 0.5,
            y=self.occupied_point[1], z=self.occupied_point[2])
        line.use_inflated = True
        line.unknown_as_occupied = False
        checked = self.call(self.line, line)
        if not checked.ready or checked.clear:
            raise RuntimeError("line through an observed occupied voxel was not rejected")
        return {
            "point": list(self.occupied_point),
            "occupancy": int(result.occupancy),
            "inflated_occupancy": int(result.inflated_occupancy),
            "has_esdf": bool(result.has_esdf),
            "esdf_distance": float(result.esdf_distance),
            "line_clear": bool(checked.clear),
        }

    def stable_preflight(self):
        topic = "/drone0/fmu/out/vehicle_status_v4"
        ready_since = None

        def stable():
            nonlocal ready_since
            status = self.latest.get(topic)
            if status is None or not status.pre_flight_checks_pass:
                ready_since = None
                return False
            ready_since = ready_since or time.monotonic()
            return time.monotonic() - ready_since >= 2.0

        self.spin_until(stable, 120, "PX4 preflight did not stabilize")

    def initialize(self, require_preflight=False):
        required = (
            "/drone0/sensor_measurements/Lidar/points",
            "/drone0/navigation/points",
        )
        self.spin_until(
            lambda: all(self.counts.get(topic, 0) >= 30 for topic in required)
            and self.latest.get("status") is not None
            and self.latest["status"].ready
            and self.latest["status"].accepted_clouds >= 30
            and self.occupied_point is not None
            and all(width > 0 for width in self.output_widths.values())
            and self.query.service_is_ready() and self.line.service_is_ready(),
            120, "real LiDAR/map/query readiness did not reach the acceptance gate")
        if require_preflight:
            self.stable_preflight()

    def platform_service(self, name, command, predicate):
        client = self.node.create_client(SetBool, "/drone0/" + name)
        self.clients.append(client)
        self.spin_until(client.service_is_ready, 20, name + " unavailable")
        self.acks.pop(command, None)
        future = client.call_async(SetBool.Request(data=True))
        self.spin_until(future.done, 10, name + " timed out")
        if not future.result().success:
            raise RuntimeError("AS2 rejected " + name)
        self.spin_until(lambda: command in self.acks, 10, "PX4 did not acknowledge " + name)
        if self.acks[command] != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            raise RuntimeError("PX4 rejected " + name)
        self.spin_until(predicate, 15, name + " feedback timed out")

    def action(self, kind, name, goal, success_field, timeout):
        client = ActionClient(self.node, kind, "/drone0/" + name)
        self.clients.append(client)
        self.spin_until(client.server_is_ready, 20, name + " unavailable")
        future = client.send_goal_async(goal)
        self.spin_until(future.done, 10, name + " request timed out")
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError("AS2 rejected " + name)
        result = handle.get_result_async()
        try:
            self.spin_until(result.done, timeout, name + " execution timed out")
        except BaseException:
            handle.cancel_goal_async()
            raise
        if not getattr(result.result().result, success_field):
            raise RuntimeError("AS2 failed " + name)

    def arm_and_offboard(self):
        mode_topic = "/drone0/fmu/out/vehicle_control_mode"
        self.platform_service(
            "set_arming_state", VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            lambda: self.latest.get(mode_topic, VehicleControlMode()).flag_armed)
        self.platform_service(
            "set_offboard_mode", VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            lambda: self.latest.get(mode_topic, VehicleControlMode()).flag_control_offboard_enabled)

    def px4_land(self):
        mode_topic = "/drone0/fmu/out/vehicle_control_mode"
        if not self.latest.get(mode_topic, VehicleControlMode()).flag_armed:
            return
        message = VehicleCommand()
        message.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        message.target_system = 72
        message.target_component = 1
        message.source_system = 255
        message.source_component = 1
        message.from_external = True
        for _ in range(10):
            message.timestamp = self.node.get_clock().now().nanoseconds // 1000
            self.vehicle_command.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.acks.get(message.command) == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                break
        self.spin_until(
            lambda: not self.latest.get(mode_topic, VehicleControlMode()).flag_armed,
            120, "PX4 emergency land did not disarm")

    def run_normal(self):
        self.initialize(require_preflight=True)
        initial_query = self.query_observed_voxel()
        accepted_before = int(self.latest["status"].accepted_clouds)
        mission_updates_before = len(self.update_stamps)
        self.arm_and_offboard()
        self.action(
            Takeoff, "TakeoffBehavior",
            Takeoff.Goal(takeoff_height=0.8, takeoff_speed=0.3),
            "takeoff_success", 90)
        self.spin_until(
            lambda: self.latest.get(
                "/drone0/platform/info", PlatformInfo()).status.state
            == PlatformStatus.FLYING,
            20, "platform did not enter FLYING after TakeoffBehavior")
        self.spin_until(lambda: len(self.poses) >= 3, 10, "canonical pose missing")
        start = self.poses[-1]
        goal = GoToWaypoint.Goal()
        goal.target_pose.header.frame_id = "earth"
        goal.target_pose.point.x = start[0] + 2.5
        goal.target_pose.point.y = start[1]
        goal.target_pose.point.z = start[2]
        goal.max_speed = 0.5
        motion_updates_before = len(self.update_stamps)
        self.action(GoToWaypoint, "GoToBehavior", goal, "go_to_success", 120)
        motion_updates_after = len(self.update_stamps)
        end = self.poses[-1]
        self.action(Land, "LandBehavior", Land.Goal(land_speed=0.3), "land_success", 90)
        mode_topic = "/drone0/fmu/out/vehicle_control_mode"
        self.spin_until(
            lambda: not self.latest.get(mode_topic, VehicleControlMode()).flag_armed,
            30, "LandBehavior completed without disarm")

        final_query = self.query_observed_voxel()
        mission_updates = self.update_stamps[mission_updates_before:]
        motion_updates = self.update_stamps[motion_updates_before:motion_updates_after]
        mission_gaps = [
            later - earlier for earlier, later
            in zip(mission_updates, mission_updates[1:])]
        gaps = [later - earlier for earlier, later in zip(motion_updates, motion_updates[1:])]
        travel = math.dist(start, end)
        result = {
            "success": travel >= 2.0 and len(motion_updates) >= 5
            and (not gaps or max(gaps) <= 0.5),
            "scenario": self.scenario,
            "real_input_counts": {topic: self.counts.get(topic, 0) for topic in (
                "/drone0/sensor_measurements/Lidar/points", "/drone0/navigation/points")},
            "accepted_clouds_before_motion": accepted_before,
            "accepted_clouds_after_motion": int(self.latest["status"].accepted_clouds),
            "mission_update_count": len(mission_updates),
            "maximum_mission_update_gap_s": max(mission_gaps) if mission_gaps else 0.0,
            "motion_update_count": len(motion_updates),
            "maximum_update_gap_s": max(gaps) if gaps else 0.0,
            "motion_distance_m": travel,
            "map_outputs_max_width": self.output_widths,
            "initial_query": initial_query,
            "final_query": final_query,
            "landed_and_disarmed": True,
        }
        if not result["success"]:
            raise RuntimeError("normal acceptance thresholds failed: " + json.dumps(result))
        self.write_result(result)

    def run_sensor_fault(self):
        self.initialize()
        accepted_before = int(self.latest["status"].accepted_clouds)
        last_update = self.stamp_seconds(self.latest["status"].last_update)
        self.marker("sensor-fault-ready.json", {"accepted_clouds": accepted_before})
        injection = self.wait_file("sensor-fault-injected.json")
        detection_started = time.monotonic()
        self.spin_until(
            lambda: self.latest.get("status") is not None and not self.latest["status"].ready,
            5, "map did not become stale after LiDAR interruption")
        detection_latency = time.monotonic() - detection_started
        stale_input_age = self.node.get_clock().now().nanoseconds / 1e9 - last_update
        request = QueryVoxel.Request()
        request.point = Point(x=0.0, y=0.0, z=0.0)
        refused = not self.call(self.query, request).ready
        self.wait_file("sensor-fault-recovered.json")
        self.spin_until(
            lambda: self.latest.get("status") is not None
            and self.latest["status"].ready
            and self.latest["status"].accepted_clouds > accepted_before,
            15, "map did not recover after fresh LiDAR resumed")
        result = {
            "success": detection_latency <= 1.5 and refused,
            "scenario": self.scenario,
            "target_container": injection.get("target_container"),
            "stale_detection_wall_s": detection_latency,
            "stale_input_age_sim_s": stale_input_age,
            "query_refused_while_stale": refused,
            "accepted_clouds_before": accepted_before,
            "accepted_clouds_after": int(self.latest["status"].accepted_clouds),
            "recovered": True,
        }
        if not result["success"]:
            raise RuntimeError("sensor fault thresholds failed: " + json.dumps(result))
        self.write_result(result)

    def run_algorithm_fault(self):
        self.initialize()
        accepted_before = int(self.latest["status"].accepted_clouds)
        self.marker("algorithm-fault-ready.json", {"accepted_clouds": accepted_before})
        injection = self.wait_file("algorithm-fault-injected.json")
        injected_at = time.monotonic()
        self.spin_until(
            lambda: time.monotonic() - self.received_at.get("status", 0.0) > 0.75,
            5, "map status continued after the map process stopped")
        unavailable_after = time.monotonic() - injected_at
        self.marker("algorithm-unavailable-observed.json")
        restarted = self.wait_file("algorithm-restarted.json")
        old_count = self.counts.get("status", 0)
        self.spin_until(
            lambda: self.counts.get("status", 0) > old_count
            and self.latest["status"].accepted_clouds == 0
            and not self.latest["status"].ready,
            5, "restarted map did not expose an empty epoch")
        self.marker("algorithm-empty-observed.json")
        self.wait_file("algorithm-input-resumed.json")
        started = time.monotonic()
        self.spin_until(
            lambda: self.latest["status"].ready
            and self.latest["status"].accepted_clouds > 0,
            15, "restarted map did not become ready from fresh scans")
        recovery = time.monotonic() - started
        result = {
            "success": unavailable_after <= 2.0 and recovery <= 15.0,
            "scenario": self.scenario,
            "target_container": injection.get("target_container"),
            "status_unavailable_after_wall_s": unavailable_after,
            "empty_epoch_observed": True,
            "recovery_after_input_wall_s": recovery,
            "accepted_clouds_before": accepted_before,
            "accepted_clouds_new_epoch": int(self.latest["status"].accepted_clouds),
            "restart_wall_time": restarted.get("wall_time"),
        }
        if not result["success"]:
            raise RuntimeError("algorithm fault thresholds failed: " + json.dumps(result))
        self.write_result(result)

    def write_result(self, result):
        (self.output / "map-status-history.json").write_text(
            json.dumps(self.status_history, indent=2) + "\n")
        (self.output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2), flush=True)

    def run(self):
        try:
            if self.scenario == "normal":
                self.run_normal()
            elif self.scenario == "sensor_fault":
                self.run_sensor_fault()
            else:
                self.run_algorithm_fault()
        except BaseException as error:
            failure = {"scenario": self.scenario, "error": repr(error), "wall_time": time.time()}
            (self.output / "failure.json").write_text(json.dumps(failure, indent=2) + "\n")
            try:
                self.px4_land()
                failure["recovery"] = "landed_and_disarmed"
            except BaseException as recovery_error:
                failure["recovery_error"] = repr(recovery_error)
            (self.output / "failure.json").write_text(json.dumps(failure, indent=2) + "\n")
            raise
        finally:
            self.node.destroy_node()


def main():
    rclpy.init()
    try:
        Acceptance().run()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
