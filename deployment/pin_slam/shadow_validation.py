#!/usr/bin/env python3
"""Run the PIN-SLAM AirSim shadow gate and emit machine-readable evidence."""

import json
import math
import os
import time
from bisect import bisect_left
from pathlib import Path

import rclpy
from as2_msgs.action import GoToWaypoint, Land, Takeoff
from as2_msgs.msg import PlatformInfo, PlatformStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    FailsafeFlags,
    VehicleCommandAck,
    VehicleControlMode,
    VehicleStatus,
)
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool

OUTPUT = Path(os.environ.get("PIN_SLAM_SHADOW_RESULTS", "/results/pin-slam-shadow"))
OUTPUT.mkdir(parents=True, exist_ok=True)
MAX_PAIRING_ERROR_S = 0.05
TAKEOFF_SPEED_MPS = 0.3
TRANSLATION_SPEED_MPS = 0.4


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


class ShadowRun:
    def __init__(self):
        self.node = rclpy.create_node(
            "pin_slam_shadow_flight",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.pose = None
        self.control = None
        self.vehicle_status = None
        self.failsafe = None
        self.acks = {}
        self.platform = None
        self.inputs = {}
        self.truth = {}
        self.candidates = {}
        self.map_stamps = set()
        self.diagnostics = {}
        self.canonical_count = 0
        self.last_stream_arrival = {}
        self.airborne_started = None
        self.airborne_duration_s = 0.0
        self.active_goal = None
        self.scenario = os.environ.get("PIN_SLAM_SCENARIO", "shadow")
        self.fault_evidence = None

        self.node.create_subscription(
            PoseStamped, "/drone0/self_localization/pose",
            self.pose_callback, qos_profile_sensor_data)
        self.node.create_subscription(
            PointCloud2, "/drone0/sensor_measurements/Lidar/points",
            self.input_callback, qos_profile_sensor_data)
        self.node.create_subscription(
            PoseStamped, "/pin_slam/validation/airsim_truth",
            self.truth_callback, qos_profile_sensor_data)
        self.node.create_subscription(
            Odometry, "/drone0/pin_slam/odometry",
            self.candidate_callback, qos_profile_sensor_data)
        self.node.create_subscription(
            PointCloud2, "/drone0/pin_slam/neural_points",
            self.map_callback, qos_profile_sensor_data)
        self.node.create_subscription(
            VehicleControlMode, "/drone0/fmu/out/vehicle_control_mode",
            lambda message: setattr(self, "control", message), qos_profile_sensor_data)
        self.node.create_subscription(
            VehicleStatus, "/drone0/fmu/out/vehicle_status_v4",
            lambda message: setattr(self, "vehicle_status", message),
            qos_profile_sensor_data)
        self.node.create_subscription(
            FailsafeFlags, "/drone0/fmu/out/failsafe_flags",
            lambda message: setattr(self, "failsafe", message), qos_profile_sensor_data)
        self.node.create_subscription(
            VehicleCommandAck, "/drone0/fmu/out/vehicle_command_ack_v1",
            self.ack_callback, qos_profile_sensor_data)
        self.node.create_subscription(
            PlatformInfo, "/drone0/platform/info",
            lambda message: setattr(self, "platform", message), qos_profile_sensor_data)
        self.node.create_subscription(
            DiagnosticArray, "/drone0/pin_slam/status", self.status_callback, 10)

    def input_callback(self, message):
        self.last_stream_arrival["lidar"] = time.monotonic()
        sample_stamp = stamp_ns(message)
        if sample_stamp in self.inputs:
            return
        self.inputs[sample_stamp] = {
            "stamp_ns": sample_stamp,
            "point_count": message.width * message.height,
        }

    def truth_callback(self, message):
        self.truth[stamp_ns(message)] = message.pose

    def candidate_callback(self, message):
        self.last_stream_arrival["candidate"] = time.monotonic()
        self.candidates[stamp_ns(message)] = message

    def pose_callback(self, message):
        self.pose = message
        self.canonical_count += 1

    def map_callback(self, message):
        if message.width > 0:
            self.map_stamps.add(stamp_ns(message))

    def status_callback(self, message):
        for item in message.status:
            if item.name != "pin_slam/runtime":
                continue
            self.diagnostics[item.message] = self.diagnostics.get(item.message, 0) + 1

    def ack_callback(self, message):
        self.acks[message.command] = {
            "result": message.result,
            "result_param1": message.result_param1,
            "result_param2": message.result_param2,
            "target_system": message.target_system,
            "target_component": message.target_component,
            "timestamp": message.timestamp,
        }

    def px4_snapshot(self):
        control_fields = (
            "timestamp",
            "flag_armed",
            "flag_control_offboard_enabled",
            "flag_control_position_enabled",
            "flag_control_velocity_enabled",
        )
        status_fields = (
            "timestamp",
            "arming_state",
            "nav_state",
            "failsafe",
            "pre_flight_checks_pass",
            "power_input_valid",
            "usb_connected",
            "open_drone_id_system_present",
            "open_drone_id_system_healthy",
        )
        failsafe_fields = (
            "timestamp",
            "angular_velocity_invalid",
            "attitude_invalid",
            "local_altitude_invalid",
            "local_position_invalid",
            "local_position_invalid_relaxed",
            "local_velocity_invalid",
            "global_position_invalid",
            "global_position_invalid_relaxed",
            "offboard_control_signal_lost",
            "home_position_invalid",
            "manual_control_signal_lost",
            "gcs_connection_lost",
            "battery_unhealthy",
            "geofence_breached",
            "navigator_failure",
            "fd_critical_failure",
            "fd_esc_arming_failure",
            "fd_motor_failure",
        )

        def fields(message, names):
            if message is None:
                return None
            return {name: getattr(message, name) for name in names}

        return {
            "wall_time_ns": time.time_ns(),
            "control": fields(self.control, control_fields),
            "vehicle_status": fields(self.vehicle_status, status_fields),
            "failsafe_flags": fields(self.failsafe, failsafe_fields),
            "command_acks": self.acks,
        }

    def write_px4_snapshot(self, stage):
        snapshot = self.px4_snapshot()
        (OUTPUT / ("px4-" + stage + ".json")).write_text(
            json.dumps(snapshot, indent=2) + "\n")
        return snapshot

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait(self, predicate, seconds=30, description="condition"):
        deadline = time.monotonic() + seconds
        while not predicate():
            if time.monotonic() > deadline:
                raise RuntimeError("timeout waiting for " + description)
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def service(self, name, value, command, feedback):
        client = self.node.create_client(SetBool, "/drone0/" + name)
        self.wait(client.service_is_ready, 20, name + " service")
        self.acks.pop(command, None)
        future = client.call_async(SetBool.Request(data=value))
        self.wait(future.done, 15, name + " service response")
        if not future.result().success:
            self.write_px4_snapshot(name + "-service-rejected")
            raise RuntimeError(name + " rejected")
        self.wait(lambda: command in self.acks, 15, name + " PX4 acknowledgement")
        acknowledgement = self.acks[command]
        if acknowledgement["result"] != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            snapshot = self.write_px4_snapshot(name + "-px4-rejected")
            raise RuntimeError(
                name + " rejected by PX4: " + json.dumps(snapshot, sort_keys=True))
        self.wait(feedback, 15, name + " firmware state")
        self.write_px4_snapshot(name + "-confirmed")

    def wait_for_preflight(self):
        self.wait(
            lambda: self.vehicle_status is not None,
            30,
            "PX4 vehicle status",
        )
        timeout = float(os.environ.get("PX4_PREFLIGHT_TIMEOUT_SECONDS", "45"))
        try:
            self.wait(
                lambda: self.vehicle_status.pre_flight_checks_pass,
                timeout,
                "PX4 preflight checks",
            )
        except Exception as error:
            snapshot = self.write_px4_snapshot("preflight-failed")
            raise RuntimeError(
                "PX4 preflight checks did not pass: "
                + json.dumps(snapshot, sort_keys=True)
            ) from error
        self.write_px4_snapshot("preflight-passed")

    def action(self, kind, name, goal, success_field, seconds=120):
        client = ActionClient(self.node, kind, "/drone0/" + name)
        self.wait(client.server_is_ready, 20)
        future = client.send_goal_async(goal)
        self.wait(future.done, 20)
        handle = future.result()
        if not handle or not handle.accepted:
            raise RuntimeError(name + " rejected")
        self.active_goal = handle
        result_future = handle.get_result_async()
        try:
            self.wait(result_future.done, seconds)
            if not getattr(result_future.result().result, success_field):
                raise RuntimeError(name + " failed")
        except Exception:
            cancel_future = handle.cancel_goal_async()
            try:
                self.wait(cancel_future.done, 10)
            except Exception:
                pass
            raise
        finally:
            self.active_goal = None

    def prepare_observation_window(self):
        self.wait(lambda: self.pose is not None, 40)
        self.wait(lambda: len(self.truth) >= 10, 20)
        self.wait(lambda: len(self.candidates) >= 2, 60)
        self.spin(1)
        self.inputs.clear()
        self.truth.clear()
        self.candidates.clear()
        self.map_stamps.clear()
        self.diagnostics.clear()
        self.canonical_count = 0
        self.last_stream_arrival.clear()
        self.spin(1)

    def execute_fault(self):
        if self.scenario not in ("sensor_fault", "algorithm_fault"):
            raise ValueError("execute_fault requires a fault scenario")
        self.prepare_observation_window()
        self.wait(lambda: self.vehicle_status is not None, 30, "PX4 vehicle status")
        self.wait(lambda: self.control is not None, 30, "PX4 control mode")
        self.write_px4_snapshot("before-offboard")
        if not self.control.flag_control_offboard_enabled:
            self.service(
                "set_offboard_mode", True, 176,
                lambda: self.control is not None
                and self.control.flag_control_offboard_enabled)
        self.wait_for_preflight()
        self.service(
            "set_arming_state", True, 400,
            lambda: self.control is not None and self.control.flag_armed)
        initial_z = self.pose.pose.position.z
        self.action(
            Takeoff, "TakeoffBehavior",
            Takeoff.Goal(takeoff_height=1.5, takeoff_speed=TAKEOFF_SPEED_MPS),
            "takeoff_success")
        self.wait(lambda: self.pose.pose.position.z > initial_z + 0.7, 30)
        self.wait(
            lambda: self.platform is not None
            and self.platform.status.state == PlatformStatus.FLYING,
            30, "platform flying state")
        self.airborne_started = time.monotonic()

        for dx, dy in ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)):
            position = self.pose.pose.position
            goal = GoToWaypoint.Goal()
            goal.target_pose.header.frame_id = "earth"
            goal.target_pose.point.x = position.x + dx
            goal.target_pose.point.y = position.y + dy
            goal.target_pose.point.z = position.z
            goal.max_speed = TRANSLATION_SPEED_MPS
            self.action(GoToWaypoint, "GoToBehavior", goal, "go_to_success", 120)
        self.wait(
            lambda: len(self.candidates) >= 10 and len(self.map_stamps) >= 1,
            60, "PIN-SLAM tracking before fault injection")

        stream = "lidar" if self.scenario == "sensor_fault" else "candidate"
        baseline = {
            "lidar_messages": len(self.inputs),
            "candidate_messages": len(self.candidates),
            "canonical_pose_samples": self.canonical_count,
        }
        (OUTPUT / "fault-ready.json").write_text(json.dumps({
            "scenario": self.scenario,
            "watched_stream": stream,
            "baseline": baseline,
            "ready_wall_time": time.time(),
        }, indent=2) + "\n")
        detection_started = time.monotonic()
        self.wait(
            lambda: stream in self.last_stream_arrival
            and time.monotonic() - self.last_stream_arrival[stream] >= 2.0,
            30, f"{stream} stream to stop after fault injection")
        detection_seconds = time.monotonic() - detection_started
        detected_wall_time = time.time()
        canonical_at_detection = self.canonical_count
        self.spin(1.0)
        canonical_after_hold = self.canonical_count
        if canonical_after_hold - canonical_at_detection < 20:
            raise RuntimeError("canonical state did not remain live after PIN-SLAM fault")

        self.airborne_duration_s = time.monotonic() - self.airborne_started
        self.action(Land, "LandBehavior", Land.Goal(land_speed=0.5), "land_success", 120)
        self.wait(
            lambda: self.control is not None and not self.control.flag_armed,
            30, "PX4 disarmed state after fault landing")
        self.write_px4_snapshot("landed")
        self.spin(3)
        self.fault_evidence = {
            "scenario": self.scenario,
            "watched_stream": stream,
            "baseline": baseline,
            "final": {
                "lidar_messages": len(self.inputs),
                "candidate_messages": len(self.candidates),
                "canonical_pose_samples": self.canonical_count,
            },
            "detection_seconds": detection_seconds,
            "recovery_seconds": time.monotonic() - detection_started,
            "detected_wall_time": detected_wall_time,
            "stream_stale_seconds": time.monotonic() - self.last_stream_arrival[stream],
            "canonical_samples_during_hold": canonical_after_hold - canonical_at_detection,
            "landed_and_disarmed": True,
        }

    def execute_flight(self):
        self.prepare_observation_window()
        self.wait(lambda: self.vehicle_status is not None, 30, "PX4 vehicle status")
        self.wait(lambda: self.control is not None, 30, "PX4 control mode")
        self.write_px4_snapshot("before-offboard")
        if not self.control.flag_control_offboard_enabled:
            self.service(
                "set_offboard_mode",
                True,
                176,
                lambda: (
                    self.control is not None
                    and self.control.flag_control_offboard_enabled
                ),
            )
        self.wait_for_preflight()
        self.service(
            "set_arming_state",
            True,
            400,
            lambda: self.control is not None and self.control.flag_armed,
        )
        initial_z = self.pose.pose.position.z
        self.action(
            Takeoff, "TakeoffBehavior",
            Takeoff.Goal(takeoff_height=1.5, takeoff_speed=TAKEOFF_SPEED_MPS),
            "takeoff_success")
        self.wait(lambda: self.pose.pose.position.z > initial_z + 0.7, 30)
        self.wait(
            lambda: self.platform is not None
            and self.platform.status.state == PlatformStatus.FLYING, 30)
        self.airborne_started = time.monotonic()
        waypoint_count = int(os.environ.get("PIN_SLAM_WAYPOINT_COUNT", "5"))
        minimum_airborne_seconds = float(
            os.environ.get("PIN_SLAM_MIN_AIRBORNE_SECONDS", "65"))
        if waypoint_count <= 0 or minimum_airborne_seconds < 0.0:
            raise ValueError("flight count and duration must be non-negative")
        for _ in range(waypoint_count):
            position = self.pose.pose.position
            goal = GoToWaypoint.Goal()
            goal.target_pose.header.frame_id = "earth"
            goal.target_pose.point.x = position.x + 1.0
            goal.target_pose.point.y = position.y
            goal.target_pose.point.z = position.z
            goal.max_speed = TRANSLATION_SPEED_MPS
            self.action(GoToWaypoint, "GoToBehavior", goal, "go_to_success", 120)
            self.spin(2)
        self.spin(max(
            0.0, minimum_airborne_seconds - (time.monotonic() - self.airborne_started)))
        self.airborne_duration_s = time.monotonic() - self.airborne_started
        self.action(Land, "LandBehavior", Land.Goal(land_speed=0.5), "land_success", 120)
        self.wait(
            lambda: self.control is not None and not self.control.flag_armed,
            30,
            "PX4 disarmed state after landing",
        )
        self.write_px4_snapshot("landed")
        self.spin(5)

    def recover(self):
        if self.active_goal is not None:
            try:
                cancel_future = self.active_goal.cancel_goal_async()
                self.wait(cancel_future.done, 10)
            except Exception:
                pass
            self.active_goal = None
        if not self.control or not self.control.flag_armed:
            return
        try:
            self.action(Land, "LandBehavior", Land.Goal(land_speed=0.5), "land_success", 60)
            self.spin(3)
        except Exception:
            pass
        if self.control and self.control.flag_armed:
            try:
                self.service(
                    "set_arming_state",
                    False,
                    400,
                    lambda: self.control is not None and not self.control.flag_armed,
                )
                self.spin(2)
            except Exception:
                pass

    def evaluate(self, result_filename="result.json"):
        output_stamps = sorted(self.candidates)
        if not output_stamps:
            raise RuntimeError("no candidate odometry received")
        input_stamps = sorted(
            stamp for stamp in self.inputs
            if output_stamps[0] <= stamp <= output_stamps[-1])
        paired_stamps = sorted(set(input_stamps).intersection(output_stamps))
        coverage = len(paired_stamps) / len(input_stamps) if input_stamps else 0.0
        gaps = [(right - left) * 1.0e-9
                for left, right in zip(output_stamps, output_stamps[1:])]
        non_monotonic = sum(gap <= 0.0 for gap in gaps)
        positive_gaps = [gap for gap in gaps if gap > 0.0]

        raw_rows = []
        pairing_errors = []
        truth_stamps = sorted(self.truth)
        accuracy_stamps = [
            stamp for stamp in paired_stamps
            if truth_stamps and truth_stamps[0] <= stamp <= truth_stamps[-1]]
        for sample_stamp in accuracy_stamps:
            if not truth_stamps:
                continue
            index = bisect_left(truth_stamps, sample_stamp)
            options = truth_stamps[max(0, index - 1):min(len(truth_stamps), index + 1)]
            truth_stamp = min(options, key=lambda value: abs(value - sample_stamp))
            pairing_error = abs(truth_stamp - sample_stamp) * 1.0e-9
            pairing_errors.append(pairing_error)
            if pairing_error > MAX_PAIRING_ERROR_S:
                continue
            message = self.candidates[sample_stamp]
            truth_pose = self.truth[truth_stamp]
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            estimate_yaw = math.atan2(
                2 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1 - 2 * (orientation.y ** 2 + orientation.z ** 2),
            )
            raw_rows.append({
                "stamp_ns": sample_stamp,
                "estimate": [position.x, position.y, position.z],
                "truth": [truth_pose.position.x, truth_pose.position.y, truth_pose.position.z],
                "estimate_yaw": estimate_yaw,
                "truth_yaw": math.atan2(
                    2 * (truth_pose.orientation.w * truth_pose.orientation.z
                         + truth_pose.orientation.x * truth_pose.orientation.y),
                    1 - 2 * (truth_pose.orientation.y ** 2
                             + truth_pose.orientation.z ** 2),
                ),
                "pairing_error_s": pairing_error,
            })
        if not raw_rows:
            raise RuntimeError("no truth/candidate pairs satisfy the time pairing gate")

        estimate_origin = raw_rows[0]["estimate"]
        truth_origin = raw_rows[0]["truth"]
        estimate_yaw_origin = raw_rows[0]["estimate_yaw"]
        truth_yaw_origin = raw_rows[0]["truth_yaw"]
        rows = []
        for sample in raw_rows:
            estimate = [sample["estimate"][i] - estimate_origin[i] for i in range(3)]
            truth = [sample["truth"][i] - truth_origin[i] for i in range(3)]
            yaw_error = math.atan2(
                math.sin((sample["estimate_yaw"] - estimate_yaw_origin)
                         - (sample["truth_yaw"] - truth_yaw_origin)),
                math.cos((sample["estimate_yaw"] - estimate_yaw_origin)
                         - (sample["truth_yaw"] - truth_yaw_origin)),
            )
            rows.append({
                "stamp_ns": sample["stamp_ns"],
                "estimate": estimate,
                "truth": truth,
                "estimate_yaw": sample["estimate_yaw"],
                "truth_yaw": sample["truth_yaw"],
                "error_m": math.dist(estimate, truth),
                "horizontal_error_m": math.dist(estimate[:2], truth[:2]),
                "yaw_error_deg": abs(math.degrees(yaw_error)),
                "pairing_error_s": sample["pairing_error_s"],
            })

        errors = sorted(row["error_m"] for row in rows)
        yaw_errors = sorted(row["yaw_error_deg"] for row in rows)
        horizontal_motion = max(
            (math.dist([0.0, 0.0], row["truth"][:2]) for row in rows), default=0.0)
        tracking_losses = (
            self.diagnostics.get("tracking_lost", 0)
            + self.diagnostics.get("processing_failed", 0)
        )
        result = {
            "scenario": self.scenario,
            "samples": len(rows),
            "airborne_duration_s": self.airborne_duration_s,
            "input_lidar_messages": len(input_stamps),
            "candidate_odom_messages": len(output_stamps),
            "paired_output_messages": len(paired_stamps),
            "neural_point_samples": len(self.map_stamps),
            "truth_max_horizontal_movement_m": horizontal_motion,
            "sample_coverage": coverage,
            "maximum_output_gap_s": max(positive_gaps, default=None),
            "maximum_pairing_error_s": max(pairing_errors, default=None),
            "pairing_gate_s": MAX_PAIRING_ERROR_S,
            "truth_messages": len(truth_stamps),
            "accuracy_candidate_messages": len(accuracy_stamps),
            "pairing_rejected_samples": len(accuracy_stamps) - len(raw_rows),
            "unmatched_output_messages": len(set(output_stamps) - set(input_stamps)),
            "non_monotonic_output_count": non_monotonic,
            "tracking_loss_events": tracking_losses,
            "diagnostics": self.diagnostics,
            "horizontal_rmse_m": math.sqrt(
                sum(row["horizontal_error_m"] ** 2 for row in rows) / len(rows)),
            "position_rmse_m": math.sqrt(
                sum(row["error_m"] ** 2 for row in rows) / len(rows)),
            "position_p95_m": errors[min(len(errors) - 1, int(len(errors) * 0.95))],
            "yaw_p95_deg": yaw_errors[min(len(yaw_errors) - 1, int(len(yaw_errors) * 0.95))],
            "landed_and_disarmed": bool(self.control and not self.control.flag_armed),
        }
        if self.fault_evidence is not None:
            result["fault"] = self.fault_evidence
        (OUTPUT / "samples.json").write_text(json.dumps(rows, indent=2) + "\n")
        (OUTPUT / "input-samples.json").write_text(
            json.dumps([self.inputs[key] for key in sorted(self.inputs)], indent=2) + "\n")
        (OUTPUT / result_filename).write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
        return result


def main():
    rclpy.init()
    run = ShadowRun()
    try:
        observe_only_seconds = float(os.environ.get("PIN_SLAM_OBSERVE_ONLY_SECONDS", "0"))
        if observe_only_seconds > 0.0:
            run.prepare_observation_window()
            run.spin(observe_only_seconds)
        elif run.scenario in ("sensor_fault", "algorithm_fault"):
            run.execute_fault()
        else:
            run.execute_flight()
        run.evaluate()
    except Exception as error:
        run.recover()
        partial_result = None
        try:
            partial_result = run.evaluate("partial-result.json")
        except Exception as evaluation_error:
            partial_result = {"evaluation_error": repr(evaluation_error)}
        failure = {
            "error": repr(error),
            "landed_and_disarmed": bool(run.control and not run.control.flag_armed),
            "px4": run.write_px4_snapshot("failure"),
            "partial_result": partial_result,
        }
        (OUTPUT / "failure.json").write_text(json.dumps(failure, indent=2) + "\n")
        raise
    finally:
        run.node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
