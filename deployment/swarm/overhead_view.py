#!/usr/bin/env python3
"""Keep an AirSim external camera aimed at the live fleet centroid."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import sys
import time
from typing import Any

try:
    from .view import camera_options, camera_pose, camera_quaternion
except ImportError:
    from view import camera_options, camera_pose, camera_quaternion


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


def centroid(points: list[tuple[float, float, float]]) -> tuple[float, float, float]:
    if not points:
        raise ValueError("at least one vehicle pose is required")
    size = float(len(points))
    return tuple(sum(point[axis] for point in points) / size for axis in range(3))


def update_camera(
    rpc: Any, vehicles: list[str], camera: dict[str, Any], origins=None
) -> tuple[float, float, float]:
    points = []
    for vehicle in vehicles:
        local = pose_position(rpc.call("simGetVehiclePose", vehicle), vehicle)
        origin = (origins or {}).get(vehicle, (0., 0., 0.))
        points.append(tuple(a + b for a, b in zip(local, origin)))
    center = centroid(points)
    geometry = camera_pose(center, camera)
    pose = {
        "position": {"x_val": geometry["X"], "y_val": geometry["Y"], "z_val": geometry["Z"]},
        "orientation": camera_quaternion(geometry),
    }
    rpc.call("simSetCameraPose", camera["name"], pose, "", True)
    return center


def run(args: argparse.Namespace) -> int:
    settings = json.loads(args.settings.read_text())
    vehicles = list(settings.get("Vehicles", {}))
    if not vehicles:
        raise ValueError("settings require at least one vehicle")
    camera = camera_options(
        {
            "name": args.camera_name,
            "mode": args.mode,
            "distance_m": args.distance,
            "depression_degrees": args.depression,
            "yaw_degrees": args.yaw,
            "altitude_m": args.altitude,
            "update_hz": args.update_hz,
        }
    )
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / "simulator/airsim/sensors"))
    try:
        from uav_as2_sensor_adapter.rpc import PersistentRpc as Rpc
    except ImportError as exc:
        raise RuntimeError("overhead tracking requires the Python msgpack package") from exc
    rpc = Rpc(args.host, int(settings.get("ApiServerPort", 41451)), args.rpc_timeout)
    interval = 1.0 / camera["update_hz"]
    consecutive_failures = 0
    updates = 0
    while not stopping:
        started = time.monotonic()
        try:
            origins = {name: tuple(float(settings["Vehicles"][name].get(axis, 0.))
                                    for axis in ("X", "Y", "Z")) for name in vehicles}
            center = update_camera(rpc, vehicles, camera, origins)
            updates += 1
            if updates == 1 or updates % 10 == 0:
                print(json.dumps({"updates": updates, "centroid_world_ned": center}), flush=True)
            consecutive_failures = 0
            if args.once:
                print(json.dumps({"vehicles": vehicles, "centroid": center}))
                return 0
        except (ConnectionError, OSError, RuntimeError) as exc:
            consecutive_failures += 1
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
    parser.add_argument("--mode", choices=("top_down", "oblique"), default="top_down")
    parser.add_argument("--distance", type=float, default=24.0)
    parser.add_argument("--depression", type=float, default=15.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--altitude", type=float, default=25.0)
    parser.add_argument("--update-hz", type=float, default=5.0)
    parser.add_argument("--rpc-timeout", type=float, default=2.0)
    parser.add_argument("--max-failures", type=int, default=10)
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
