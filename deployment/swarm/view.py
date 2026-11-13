"""AirSim overhead observation-view helpers."""
from __future__ import annotations

import copy
import math
from typing import Any


DEFAULT_CAMERA = {
    "name": "swarm_overhead",
    "altitude_m": 25.0,
    "fov_degrees": 90.0,
    "update_hz": 5.0,
    "primary_view": False,
}


def camera_options(value: dict[str, Any] | None = None) -> dict[str, Any]:
    result = dict(DEFAULT_CAMERA)
    if value:
        result.update(value)
    if not isinstance(result["name"], str) or not result["name"]:
        raise ValueError("airsim.camera.name must be a nonempty string")
    for key in ("altitude_m", "fov_degrees", "update_hz"):
        result[key] = float(result[key])
        if result[key] <= 0:
            raise ValueError(f"airsim.camera.{key} must be positive")
    if not isinstance(result["primary_view"], bool):
        raise ValueError("airsim.camera.primary_view must be boolean")
    return result


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
    """Return settings with an initial and externally trackable top-down view."""
    result = copy.deepcopy(settings)
    camera = camera_options(options)
    center_x, center_y, center_z = vehicle_centroid(result)
    camera_pose = {
        "X": center_x,
        "Y": center_y,
        "Z": center_z - camera["altitude_m"],
        "Pitch": -90.0,
        "Roll": 0.0,
        "Yaw": 0.0,
    }
    result["ViewMode"] = "Fpv" if camera["primary_view"] else "Manual"
    result["CameraDirector"] = dict(camera_pose)
    if camera["primary_view"]:
        primary_name = next(iter(result["Vehicles"]))
        primary = result["Vehicles"][primary_name]
        yaw = float(primary.get("Yaw", 0.0))
        delta_x = center_x - float(primary.get("X", 0.0))
        delta_y = center_y - float(primary.get("Y", 0.0))
        radians = math.radians(-yaw)
        relative_x = math.cos(radians) * delta_x - math.sin(radians) * delta_y
        relative_y = math.sin(radians) * delta_x + math.cos(radians) * delta_y
        primary_camera = primary.setdefault("Cameras", {}).setdefault("0", {})
        primary_camera.update(
            {
                "X": relative_x,
                "Y": relative_y,
                "Z": camera_pose["Z"] - float(primary.get("Z", 0.0)),
                "Pitch": -90.0,
                "Roll": 0.0,
                "Yaw": -yaw,
            }
        )
    external = result.setdefault("ExternalCameras", {})
    camera_settings = external.setdefault(camera["name"], {})
    camera_settings.update(camera_pose)
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
