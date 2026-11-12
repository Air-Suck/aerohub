#!/usr/bin/env python3
"""Keep an AirSim external camera above the live fleet centroid."""
from __future__ import annotations

import argparse
import json
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


def centroid(points: list[tuple[float, float, float]]) -> tuple[float, float, float]:
    if not points:
        raise ValueError("at least one vehicle pose is required")
    size = float(len(points))
    return tuple(sum(point[axis] for point in points) / size for axis in range(3))


def update_camera(
    rpc: Any, vehicles: list[str], camera: dict[str, Any], pose_source: str = "vehicle"
) -> tuple[float, float, float]:
    method = "simGetObjectPose" if pose_source == "object" else "simGetVehiclePose"
    center = centroid(
        [pose_position(rpc.call(method, vehicle), vehicle) for vehicle in vehicles]
    )
    pose = {
        "position": {
            "x_val": center[0],
            "y_val": center[1],
            "z_val": center[2] - camera["altitude_m"],
        },
        "orientation": downward_quaternion(),
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
    while not stopping:
        started = time.monotonic()
        try:
            center = update_camera(rpc, vehicles, camera, args.pose_source)
            updates += 1
            if updates % 25 == 0:
                print(json.dumps({"updates": updates, "centroid": center, "pose_source": args.pose_source}), flush=True)
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
    parser.add_argument("--altitude", type=float, default=25.0)
    parser.add_argument("--update-hz", type=float, default=5.0)
    parser.add_argument("--rpc-timeout", type=float, default=2.0)
    parser.add_argument("--max-failures", type=int, default=10)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--pose-source", choices=("vehicle", "object"), default="vehicle",
                        help="object uses global actor coordinates for named vehicles at nonzero spawn")
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
