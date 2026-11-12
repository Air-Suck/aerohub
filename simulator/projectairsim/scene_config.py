"""Apply the simple YAML sensor settings to an upstream-resolved scene."""
from copy import deepcopy
import math

CAPTURES = {"rgb": 0, "depth_planar": 1, "depth_perspective": 2, "segmentation": 5}
FIELDS = {"channels": "number-of-channels", "range": "range",
          "points_per_second": "points-per-second", "rotation_hz": "horizontal-rotation-frequency"}


def configure_sensors(scene, options, lidar_template=None):
    scene = deepcopy(scene)
    actors = {actor["name"]: actor for actor in scene["actors"] if actor.get("type") == "robot"}
    for name, sensors in options.items():
        robot = actors[name]["robot-config"]
        native = {s["id"]: s for s in robot["sensors"]}
        for sid, config in sensors.items():
            unknown = set(config) - {"type", "enabled", "position", "rotation", "width", "height", "fov", "captures", *FIELDS}
            if unknown:
                raise ValueError(f"{name}/{sid}: unknown sensor options {sorted(unknown)}")
            if sid not in native:
                if config.get("type") != "lidar" or lidar_template is None:
                    raise ValueError(f"{name}/{sid}: sensor does not exist in native scene")
                sensor = deepcopy(lidar_template)
                sensor["id"] = sid
                robot["sensors"].append(sensor)
                native[sid] = sensor
            sensor = native[sid]
            if "type" in config and config["type"] != sensor["type"]:
                raise ValueError(f"{name}/{sid}: sensor type differs from native scene")
            if "enabled" in config:
                if not isinstance(config["enabled"], bool):
                    raise ValueError(f"{sid}: enabled must be boolean")
                sensor["enabled"] = config["enabled"]
            for key, native_key in (("position", "xyz"), ("rotation", "rpy-deg")):
                if key in config:
                    values = config[key]
                    if not isinstance(values, list) or len(values) != 3 or not all(
                            isinstance(v, (int, float)) and math.isfinite(v) for v in values):
                        raise ValueError(f"{sid}: {key} requires three finite numbers")
                    sensor.setdefault("origin", {})[native_key] = " ".join(map(str, values))
            if any(key in config for key in ("width", "height", "fov", "captures")):
                if sensor["type"] != "camera":
                    raise ValueError(f"{sid}: camera settings require a camera")
                captures = config.get("captures")
                if captures is not None:
                    if not isinstance(captures, list) or not captures or set(captures) - CAPTURES.keys():
                        raise ValueError(f"{sid}: captures must select {list(CAPTURES)}")
                    selected = {CAPTURES[k] for k in captures}
                    existing = {c["image-type"]: c for c in sensor["capture-settings"]}
                    for image_type in selected - existing.keys():
                        capture = deepcopy(sensor["capture-settings"][0])
                        capture["image-type"] = image_type
                        sensor["capture-settings"].append(capture)
                    for capture in sensor["capture-settings"]:
                        capture["capture-enabled"] = capture["image-type"] in selected
                for capture in sensor["capture-settings"]:
                    for key, native_key in (("width", "width"), ("height", "height"), ("fov", "fov-degrees")):
                        if key in config:
                            value = config[key]
                            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                                raise ValueError(f"{sid}: invalid {key}")
                            if key in ("width", "height") and (isinstance(value, bool) or int(value) != value):
                                raise ValueError(f"{sid}: {key} must be an integer")
                            if key == "fov" and value >= 180:
                                raise ValueError(f"{sid}: fov must be below 180 degrees")
                            capture[native_key] = value
            for key, native_key in FIELDS.items():
                if key in config:
                    if sensor["type"] != "lidar":
                        raise ValueError(f"{sid}: {key} requires a lidar")
                    value = config[key]
                    if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                        raise ValueError(f"{sid}: invalid {key}")
                    sensor[native_key] = value
    return scene
