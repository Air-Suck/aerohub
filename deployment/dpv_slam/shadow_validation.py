#!/usr/bin/env python3
"""Run the DPV-SLAM AirSim shadow gate and emit machine-readable evidence."""

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
from px4_msgs.msg import VehicleControlMode
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_srvs.srv import SetBool

OUTPUT = Path(os.environ.get("DPV_SLAM_SHADOW_RESULTS", "/results/dpv-slam-shadow"))
OUTPUT.mkdir(parents=True, exist_ok=True)
MAX_PAIRING_ERROR_S = 0.05


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def rate_hz(stamps):
    ordered = sorted(set(stamps))
    if len(ordered) < 2:
        return 0.0
    duration = (ordered[-1] - ordered[0]) * 1.0e-9
    return (len(ordered) - 1) / duration if duration > 0.0 else 0.0


class ShadowRun:
    def __init__(self):
        self.node = rclpy.create_node(
            "dpv_slam_shadow_flight",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.pose = None
        self.canonical_stamps = []
        self.control = None
        self.platform = None
        self.images = {}
        self.camera_info_stamps = set()
        self.truth = {}
        self.candidates = {}
        self.candidate_arrivals = []
        self.sparse_samples = {}
        self.diagnostics = {}
        self.alignment_scale = None
        self.alignment_rmse = None
        self.airborne_started = None
        self.airborne_duration_s = 0.0
        self.active_goal = None

        self.node.create_subscription(
            PoseStamped, "/drone0/self_localization/pose",
            self.pose_callback, qos_profile_sensor_data)
        self.node.create_subscription(
            Image, "/drone0/sensor_measurements/FrontCamera/rgb/image",
            self.image_callback, qos_profile_sensor_data)
        self.node.create_subscription(
            CameraInfo, "/drone0/sensor_measurements/FrontCamera/rgb/camera_info",
            lambda message: self.camera_info_stamps.add(stamp_ns(message)),
            qos_profile_sensor_data)
        self.node.create_subscription(
            PoseStamped, "/dpv_slam/validation/airsim_truth",
            self.truth_callback, qos_profile_sensor_data)
        self.node.create_subscription(
            Odometry, "/drone0/dpv_slam/odometry",
            self.candidate_callback, qos_profile_sensor_data)
        self.node.create_subscription(
            PointCloud2, "/drone0/dpv_slam/sparse_points",
            self.sparse_callback, qos_profile_sensor_data)
        self.node.create_subscription(
            VehicleControlMode, "/drone0/fmu/out/vehicle_control_mode",
            lambda message: setattr(self, "control", message), qos_profile_sensor_data)
        self.node.create_subscription(
            PlatformInfo, "/drone0/platform/info",
            lambda message: setattr(self, "platform", message), qos_profile_sensor_data)
        self.node.create_subscription(
            DiagnosticArray, "/drone0/dpv_slam/status", self.status_callback, 10)

    def image_callback(self, message):
        sample_stamp = stamp_ns(message)
        if sample_stamp not in self.images:
            self.images[sample_stamp] = {
                "stamp_ns": sample_stamp,
                "width": message.width,
                "height": message.height,
                "encoding": message.encoding,
                "bytes": len(message.data),
            }

    def pose_callback(self, message):
        self.pose = message
        self.canonical_stamps.append(stamp_ns(message))

    def truth_callback(self, message):
        self.truth[stamp_ns(message)] = message.pose

    def candidate_callback(self, message):
        sample_stamp = stamp_ns(message)
        self.candidate_arrivals.append(sample_stamp)
        self.candidates[sample_stamp] = message

    def sparse_callback(self, message):
        if message.width * message.height > 0:
            self.sparse_samples[stamp_ns(message)] = message.width * message.height

    def status_callback(self, message):
        for item in message.status:
            if item.name != "dpv_slam/runtime":
                continue
            self.diagnostics[item.message] = self.diagnostics.get(item.message, 0) + 1
            values = {value.key: value.value for value in item.values}
            if item.message == "tracking":
                try:
                    self.alignment_scale = float(values["scale"])
                    self.alignment_rmse = float(values["alignment_rmse"])
                except (KeyError, ValueError):
                    pass

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait(self, predicate, seconds=30):
        deadline = time.monotonic() + seconds
        while not predicate():
            if time.monotonic() > deadline:
                raise RuntimeError("timeout waiting for condition")
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def service(self, name, value):
        client = self.node.create_client(SetBool, "/drone0/" + name)
        self.wait(client.service_is_ready, 20)
        future = client.call_async(SetBool.Request(data=value))
        self.wait(future.done, 15)
        if not future.result().success:
            raise RuntimeError(name + " rejected")

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
        self.wait(lambda: len(self.images) >= 10, 20)
        self.wait(
            lambda: len(set(self.images).intersection(self.camera_info_stamps)) >= 10, 20)
        self.spin(1)
        self.images.clear()
        self.camera_info_stamps.clear()
        self.truth.clear()
        self.candidates.clear()
        self.candidate_arrivals.clear()
        self.sparse_samples.clear()
        self.diagnostics.clear()
        self.canonical_stamps.clear()
        self.spin(1)

    def execute_flight(self):
        self.prepare_observation_window()
        self.service("set_arming_state", True)
        if not (self.control and self.control.flag_control_offboard_enabled):
            self.service("set_offboard_mode", True)
        initial_z = self.pose.pose.position.z
        self.action(
            Takeoff, "TakeoffBehavior",
            Takeoff.Goal(takeoff_height=1.5, takeoff_speed=0.5), "takeoff_success")
        self.wait(lambda: self.pose.pose.position.z > initial_z + 0.7, 30)
        self.wait(
            lambda: self.platform is not None
            and self.platform.status.state == PlatformStatus.FLYING, 30)
        self.airborne_started = time.monotonic()
        waypoint_count = int(os.environ.get("DPV_SLAM_WAYPOINT_COUNT", "8"))
        minimum_airborne_seconds = float(
            os.environ.get("DPV_SLAM_MIN_AIRBORNE_SECONDS", "65"))
        if waypoint_count <= 0 or minimum_airborne_seconds < 0.0:
            raise ValueError("flight count and duration must be non-negative")
        offsets = ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0))
        for index in range(waypoint_count):
            position = self.pose.pose.position
            dx, dy = offsets[index % len(offsets)]
            goal = GoToWaypoint.Goal()
            goal.target_pose.header.frame_id = "earth"
            goal.target_pose.point.x = position.x + dx
            goal.target_pose.point.y = position.y + dy
            goal.target_pose.point.z = position.z
            goal.max_speed = 0.5
            self.action(GoToWaypoint, "GoToBehavior", goal, "go_to_success", 120)
            self.spin(2)
        self.spin(max(
            0.0, minimum_airborne_seconds - (time.monotonic() - self.airborne_started)))
        self.airborne_duration_s = time.monotonic() - self.airborne_started
        self.action(Land, "LandBehavior", Land.Goal(land_speed=0.5), "land_success", 120)
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
                self.service("set_arming_state", False)
                self.spin(2)
            except Exception:
                pass

    def evaluate(self, result_filename="result.json"):
        output_stamps = sorted(self.candidates)
        if not output_stamps:
            raise RuntimeError("no candidate odometry received")
        image_stamps = sorted(
            stamp for stamp in self.images
            if output_stamps[0] <= stamp <= output_stamps[-1])
        matched_info = set(image_stamps).intersection(self.camera_info_stamps)
        candidate_input_pairs = set(output_stamps).intersection(image_stamps)
        coverage = len(candidate_input_pairs) / len(output_stamps)
        gaps = [(right - left) * 1.0e-9
                for left, right in zip(output_stamps, output_stamps[1:])]
        non_monotonic = sum(
            right <= left for left, right in
            zip(self.candidate_arrivals, self.candidate_arrivals[1:]))

        raw_rows = []
        pairing_errors = []
        truth_stamps = sorted(self.truth)
        for sample_stamp in output_stamps:
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
                1 - 2 * (orientation.y ** 2 + orientation.z ** 2))
            raw_rows.append({
                "stamp_ns": sample_stamp,
                "estimate": [position.x, position.y, position.z],
                "truth": [truth_pose.position.x, truth_pose.position.y, truth_pose.position.z],
                "estimate_yaw": estimate_yaw,
                "truth_yaw": math.atan2(
                    2 * (truth_pose.orientation.w * truth_pose.orientation.z
                         + truth_pose.orientation.x * truth_pose.orientation.y),
                    1 - 2 * (truth_pose.orientation.y ** 2
                             + truth_pose.orientation.z ** 2)),
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
                         - (sample["truth_yaw"] - truth_yaw_origin)))
            rows.append({
                "stamp_ns": sample["stamp_ns"],
                "estimate": estimate,
                "truth": truth,
                "error_m": math.dist(estimate, truth),
                "horizontal_error_m": math.dist(estimate[:2], truth[:2]),
                "yaw_error_deg": abs(math.degrees(yaw_error)),
                "pairing_error_s": sample["pairing_error_s"],
            })

        errors = sorted(row["error_m"] for row in rows)
        yaw_errors = sorted(row["yaw_error_deg"] for row in rows)
        horizontal_displacement = max(
            (math.dist([0.0, 0.0], row["truth"][:2]) for row in rows), default=0.0)
        horizontal_motion = sum(
            math.dist(left["truth"][:2], right["truth"][:2])
            for left, right in zip(rows, rows[1:]))
        tracking_losses = (
            self.diagnostics.get("tracking_lost", 0)
            + self.diagnostics.get("processing_failed", 0)
            + self.diagnostics.get("alignment_rejected", 0))
        result = {
            "samples": len(rows),
            "airborne_duration_s": self.airborne_duration_s,
            "image_messages": len(image_stamps),
            "camera_info_messages": len(matched_info),
            "candidate_odom_messages": len(output_stamps),
            "candidate_input_pairs": len(candidate_input_pairs),
            "sparse_point_samples": len(self.sparse_samples),
            "sparse_point_max_count": max(self.sparse_samples.values(), default=0),
            "truth_max_horizontal_movement_m": horizontal_motion,
            "truth_max_horizontal_displacement_m": horizontal_displacement,
            "image_input_rate_hz": rate_hz(image_stamps),
            "candidate_output_rate_hz": rate_hz(output_stamps),
            "canonical_pose_rate_hz": rate_hz(self.canonical_stamps),
            "canonical_pose_maximum_gap_s": max(
                ((right - left) * 1.0e-9 for left, right in
                 zip(sorted(set(self.canonical_stamps)),
                     sorted(set(self.canonical_stamps))[1:])),
                default=None),
            "sample_coverage": coverage,
            "maximum_output_gap_s": max(gaps, default=None),
            "maximum_pairing_error_s": max(pairing_errors, default=None),
            "pairing_gate_s": MAX_PAIRING_ERROR_S,
            "truth_messages": len(truth_stamps),
            "pairing_rejected_samples": len(output_stamps) - len(raw_rows),
            "non_monotonic_output_count": non_monotonic,
            "tracking_loss_events": tracking_losses,
            "diagnostics": self.diagnostics,
            "alignment_scale": self.alignment_scale,
            "alignment_rmse_m": self.alignment_rmse,
            "horizontal_rmse_m": math.sqrt(
                sum(row["horizontal_error_m"] ** 2 for row in rows) / len(rows)),
            "position_rmse_m": math.sqrt(
                sum(row["error_m"] ** 2 for row in rows) / len(rows)),
            "position_p95_m": errors[min(len(errors) - 1, int(len(errors) * 0.95))],
            "yaw_p95_deg": yaw_errors[min(len(yaw_errors) - 1, int(len(yaw_errors) * 0.95))],
            "landed_and_disarmed": bool(self.control and not self.control.flag_armed),
        }
        (OUTPUT / "samples.json").write_text(json.dumps(rows, indent=2) + "\n")
        (OUTPUT / "input-samples.json").write_text(
            json.dumps([self.images[key] for key in sorted(self.images)], indent=2) + "\n")
        (OUTPUT / result_filename).write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
        return result


def main():
    rclpy.init()
    run = ShadowRun()
    try:
        observe_only_seconds = float(os.environ.get("DPV_SLAM_OBSERVE_ONLY_SECONDS", "0"))
        if observe_only_seconds > 0.0:
            run.prepare_observation_window()
            run.spin(observe_only_seconds)
        else:
            run.execute_flight()
        run.evaluate()
    except Exception as error:
        run.recover()
        try:
            partial_result = run.evaluate("partial-result.json")
        except Exception as evaluation_error:
            partial_result = {"evaluation_error": repr(evaluation_error)}
        failure = {
            "error": repr(error),
            "landed_and_disarmed": bool(run.control and not run.control.flag_armed),
            "partial_result": partial_result,
        }
        (OUTPUT / "failure.json").write_text(json.dumps(failure, indent=2) + "\n")
        raise
    finally:
        run.node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
