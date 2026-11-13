"""AirSim fleet observation camera geometry."""
from __future__ import annotations

import copy
import math
from typing import Any


DEFAULT_CAMERA = {
    "name": "swarm_overhead",
    "mode": "top_down",
    "distance_m": 24.0,
    "depression_degrees": 15.0,
    "yaw_degrees": 0.0,
    "altitude_m": 25.0,
    "fov_degrees": 90.0,
    "update_hz": 5.0,
}


def camera_options(value: dict[str, Any] | None = None) -> dict[str, Any]:
    result = dict(DEFAULT_CAMERA)
    if value:
        result.update(value)
    if not isinstance(result["name"], str) or not result["name"]:
        raise ValueError("airsim.camera.name must be a nonempty string")
    if result["mode"] not in ("top_down", "oblique"):
        raise ValueError("airsim.camera.mode must be top_down or oblique")
    for key in ("altitude_m", "fov_degrees", "update_hz", "distance_m", "depression_degrees"):
        result[key] = float(result[key])
        if not math.isfinite(result[key]) or result[key] <= 0:
            raise ValueError(f"airsim.camera.{key} must be positive")
    result["yaw_degrees"] = float(result["yaw_degrees"])
    if not math.isfinite(result["yaw_degrees"]):
        raise ValueError("airsim.camera.yaw_degrees must be finite")
    if result["depression_degrees"] > 90 or result["fov_degrees"] >= 180:
        raise ValueError("camera depression must be <= 90 and FOV must be < 180")
    return result


def camera_pose(center, options=None) -> dict[str, float]:
    """Place the optical axis through the centroid in AirSim world NED."""
    camera = camera_options(options)
    x, y, z = center
    if camera["mode"] == "top_down":
        return {"X": x, "Y": y, "Z": z - camera["altitude_m"],
                "Pitch": -90.0, "Roll": 0.0, "Yaw": 0.0}
    yaw = math.radians(camera["yaw_degrees"])
    pitch = math.radians(camera["depression_degrees"])
    distance = camera["distance_m"]
    return {"X": x - distance * math.cos(yaw),
            "Y": y - distance * math.sin(yaw),
            "Z": z - distance * math.tan(pitch),
            "Pitch": -camera["depression_degrees"],
            "Roll": 0.0, "Yaw": camera["yaw_degrees"]}


def camera_quaternion(pose) -> dict[str, float]:
    pitch = math.radians(pose["Pitch"]) / 2
    yaw = math.radians(pose["Yaw"]) / 2
    return {"w_val": math.cos(yaw) * math.cos(pitch),
            "x_val": -math.sin(yaw) * math.sin(pitch),
            "y_val": math.cos(yaw) * math.sin(pitch),
            "z_val": math.sin(yaw) * math.cos(pitch)}


def vehicle_centroid(settings: dict[str, Any]) -> tuple[float, float, float]:
    vehicles = settings.get("Vehicles")
    if not isinstance(vehicles, dict) or not vehicles:
        raise ValueError("AirSim settings require at least one vehicle")
    coordinates = []
    for name, vehicle in vehicles.items():
        if not isinstance(vehicle, dict):
            raise ValueError(f"{name}: vehicle settings must be a mapping")
        coordinates.append(tuple(float(vehicle.get(axis, 0.0)) for axis in ("X", "Y", "Z")))
    count = float(len(coordinates))
    return tuple(sum(point[axis] for point in coordinates) / count for axis in range(3))


def apply_overhead_view(
    settings: dict[str, Any], options: dict[str, Any] | None = None
) -> dict[str, Any]:
    """Return settings with an initial and externally trackable fleet view."""
    result = copy.deepcopy(settings)
    camera = camera_options(options)
    pose = camera_pose(vehicle_centroid(result), camera)
    result["ViewMode"] = "Manual"
    result["CameraDirector"] = dict(pose)
    external = result.setdefault("ExternalCameras", {})
    camera_settings = external.setdefault(camera["name"], {})
    camera_settings.update(pose)
    camera_settings["CaptureSettings"] = [
        {
            "ImageType": 0,
            "Width": 1024,
            "Height": 1024,
            "FOV_Degrees": camera["fov_degrees"],
        }
    ]
    windows = [
        window for window in result.get("SubWindows", [])
        if not (window.get("WindowID") == 0 or window.get("CameraName") == camera["name"])
    ]
    windows.append(
        {
            "WindowID": 0,
            "CameraName": camera["name"],
            "ImageType": 0,
            "VehicleName": "",
            "External": True,
            "Visible": True,
        }
    )
    result["SubWindows"] = windows
    return result


def downward_quaternion() -> dict[str, float]:
    half_pitch = math.radians(-90.0) / 2.0
    return {
        "w_val": math.cos(half_pitch),
        "x_val": 0.0,
        "y_val": math.sin(half_pitch),
        "z_val": 0.0,
    }
