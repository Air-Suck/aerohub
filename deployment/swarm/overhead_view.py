#!/usr/bin/env python3
"""Keep an AirSim external camera above the live fleet centroid."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import sys
import time
from typing import Any

try:
    from .view import camera_options, downward_quaternion
except ImportError:
    from view import camera_options, downward_quaternion


stopping = False


def request_stop(_signum: int, _frame: Any) -> None:
    global stopping
    stopping = True


def pose_position(pose: Any, vehicle: str) -> tuple[float, float, float]:
    if not isinstance(pose, dict) or not isinstance(pose.get("position"), dict):
        raise RuntimeError(f"{vehicle}: simGetVehiclePose returned an invalid pose")
    position = pose["position"]
    try:
        return tuple(float(position[key]) for key in ("x_val", "y_val", "z_val"))
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"{vehicle}: pose position is incomplete") from exc


def pose_orientation(pose: Any, vehicle: str) -> tuple[float, float, float, float]:
    if not isinstance(pose, dict) or not isinstance(pose.get("orientation"), dict):
        raise RuntimeError(f"{vehicle}: pose orientation is invalid")
    orientation = pose["orientation"]
    try:
        values = tuple(
            float(orientation[key]) for key in ("w_val", "x_val", "y_val", "z_val")
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"{vehicle}: pose orientation is incomplete") from exc
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-9:
        raise RuntimeError(f"{vehicle}: pose orientation has zero norm")
    return tuple(value / norm for value in values)


def quaternion_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def quaternion_inverse(
    value: tuple[float, float, float, float]
) -> tuple[float, float, float, float]:
    return (value[0], -value[1], -value[2], -value[3])


def rotate_vector(
    rotation: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    rotated = quaternion_multiply(
        quaternion_multiply(rotation, (0.0, *vector)), quaternion_inverse(rotation)
    )
    return rotated[1], rotated[2], rotated[3]


def pose_dict(
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float] | dict[str, float],
) -> dict[str, dict[str, float]]:
    if isinstance(orientation, dict):
        orientation_dict = orientation
    else:
        orientation_dict = dict(
            zip(("w_val", "x_val", "y_val", "z_val"), orientation)
        )
    return {
        "position": dict(zip(("x_val", "y_val", "z_val"), position)),
        "orientation": orientation_dict,
    }


def centroid(points: list[tuple[float, float, float]]) -> tuple[float, float, float]:
    if not points:
        raise ValueError("at least one vehicle pose is required")
    size = float(len(points))
    return tuple(sum(point[axis] for point in points) / size for axis in range(3))


def update_camera(
    rpc: Any,
    vehicles: list[str],
    camera: dict[str, Any],
    primary_vehicle: str | None = None,
) -> tuple[float, float, float]:
    vehicle_poses = {
        vehicle: rpc.call("simGetVehiclePose", vehicle) for vehicle in vehicles
    }
    positions = {
        vehicle: pose_position(pose, vehicle) for vehicle, pose in vehicle_poses.items()
    }
    center = centroid(list(positions.values()))
    camera_position = (center[0], center[1], center[2] - camera["altitude_m"])
    world_orientation = downward_quaternion()
    rpc.call(
        "simSetCameraPose",
        camera["name"],
        pose_dict(camera_position, world_orientation),
        "",
        True,
    )
    if primary_vehicle is not None:
        anchor_position = positions[primary_vehicle]
        anchor_orientation = pose_orientation(vehicle_poses[primary_vehicle], primary_vehicle)
        inverse = quaternion_inverse(anchor_orientation)
        relative_position = rotate_vector(
            inverse,
            tuple(camera_position[axis] - anchor_position[axis] for axis in range(3)),
        )
        relative_orientation = quaternion_multiply(
            inverse,
            tuple(world_orientation[key] for key in ("w_val", "x_val", "y_val", "z_val")),
        )
        rpc.call(
            "simSetCameraPose",
            "0",
            pose_dict(relative_position, relative_orientation),
            primary_vehicle,
            False,
        )
    return center


def save_tracker_state(path: Path, state: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(state, indent=2) + "\n")
    temporary.replace(path)


def run(args: argparse.Namespace) -> int:
    settings = json.loads(args.settings.read_text())
    vehicles = list(settings.get("Vehicles", {}))
    if not vehicles:
        raise ValueError("settings require at least one vehicle")
    camera = camera_options(
        {
            "name": args.camera_name,
            "altitude_m": args.altitude,
            "update_hz": args.update_hz,
        }
    )
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / "simulator/airsim/sensors"))
    try:
        from uav_as2_sensor_adapter.rpc import Rpc
    except ImportError as exc:
        raise RuntimeError("overhead tracking requires the Python msgpack package") from exc
    rpc = Rpc(args.host, int(settings.get("ApiServerPort", 41451)), args.rpc_timeout)
    interval = 1.0 / camera["update_hz"]
    consecutive_failures = 0
    updates = 0
    first_center = None
    maximum_displacement = 0.0
    started_unix_s = time.time()
    primary_vehicle = args.primary_vehicle_camera
    if primary_vehicle is not None and primary_vehicle not in vehicles:
        raise ValueError(f"primary vehicle camera is not configured: {primary_vehicle}")
    while not stopping:
        started = time.monotonic()
        try:
            center = update_camera(rpc, vehicles, camera, primary_vehicle)
            updates += 1
            consecutive_failures = 0
            if first_center is None:
                first_center = center
            displacement = math.dist(first_center, center)
            maximum_displacement = max(maximum_displacement, displacement)
            state = {
                "schema_version": 1,
                "vehicles": vehicles,
                "camera": camera["name"],
                "primary_vehicle_camera": primary_vehicle,
                "started_unix_s": started_unix_s,
                "last_update_unix_s": time.time(),
                "update_count": updates,
                "first_centroid": first_center,
                "latest_centroid": center,
                "latest_camera_position": (
                    center[0], center[1], center[2] - camera["altitude_m"]
                ),
                "centroid_displacement_m": displacement,
                "max_centroid_displacement_m": maximum_displacement,
                "consecutive_failures": 0,
                "last_error": None,
            }
            if args.state_file:
                save_tracker_state(args.state_file, state)
            if args.once:
                print(json.dumps(state))
                return 0
        except (ConnectionError, OSError, RuntimeError) as exc:
            consecutive_failures += 1
            if args.state_file:
                save_tracker_state(
                    args.state_file,
                    {
                        "schema_version": 1,
                        "vehicles": vehicles,
                        "camera": camera["name"],
                        "primary_vehicle_camera": primary_vehicle,
                        "started_unix_s": started_unix_s,
                        "last_update_unix_s": None,
                        "update_count": updates,
                        "consecutive_failures": consecutive_failures,
                        "last_error": str(exc),
                    },
                )
            print(
                f"overhead view RPC failure {consecutive_failures}/{args.max_failures}: {exc}",
                file=sys.stderr,
                flush=True,
            )
            if consecutive_failures >= args.max_failures:
                raise RuntimeError("overhead view exceeded the RPC failure limit") from exc
        remaining = interval - (time.monotonic() - started)
        if remaining > 0:
            time.sleep(remaining)
    print(json.dumps({"stopped": True, "updates": updates}))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--settings", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--camera-name", default="swarm_overhead")
    parser.add_argument("--altitude", type=float, default=25.0)
    parser.add_argument("--update-hz", type=float, default=5.0)
    parser.add_argument("--rpc-timeout", type=float, default=2.0)
    parser.add_argument("--max-failures", type=int, default=10)
    parser.add_argument("--state-file", type=Path)
    parser.add_argument("--primary-vehicle-camera")
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()
    if args.max_failures < 1:
        raise ValueError("max-failures must be positive")
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    return run(args)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ImportError, OSError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
        raise SystemExit(str(exc))
