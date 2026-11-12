"""Describe supported native sensors in the existing project ROS contract."""
from pathlib import Path
import math
import re
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "airsim/sensors"))
from uav_as2_sensor_adapter.conversion import extrinsic

# Native ProjectAirSim image-type numbers match the existing AirSim contract.
IMAGES = {0: ("scene_camera", "rgb"), 1: ("depth_planar_camera", "depth_planar"),
          2: ("depth_camera", "depth_perspective"),
          5: ("segmentation_camera", "segmentation")}


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", value):
        raise ValueError(f"invalid ROS identifier: {value!r}")
    return value


def pose(config):
    origin = config.get("origin", {})
    if set(origin) - {"xyz", "rpy-deg"}:
        raise ValueError("only xyz/rpy-deg sensor origins are supported")
    xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in origin.get("rpy-deg", "0 0 0").split()]
    if len(xyz) != 3 or len(rpy) != 3:
        raise ValueError("sensor origin requires three xyz/rpy-deg values")
    if not all(math.isfinite(v) for v in xyz + rpy):
        raise ValueError("sensor origin must be finite")
    return dict(zip(("X", "Y", "Z", "Roll", "Pitch", "Yaw"), xyz + rpy))


def streams(scene, runtime, topic_root="/ProjectAirsim"):
    if not topic_root.startswith("/") or topic_root.endswith("/"):
        raise ValueError("ROS topic root must be absolute without trailing slash")
    result = []
    namespaces = set()
    actors = {a["name"]: a for a in scene["actors"] if a.get("type") == "robot"}
    for vehicle in runtime["vehicles"]:
        name, ns = identifier(vehicle["name"]), identifier(vehicle["namespace"])
        if ns in namespaces:
            raise ValueError(f"duplicate sensor namespace: {ns}")
        namespaces.add(ns)
        robot = actors[name]["robot-config"]
        # Only the rigid body link is supported. Joint chains and gimbals need
        # their actual dynamic transforms instead of a fabricated static TF.
        body = robot["links"][0]
        body_name = body["name"]
        if any(pose(body).values()):
            raise ValueError("nonidentity root link is not supported")
        source = f"{topic_root}/{scene['id']}/robots/{name}/sensors"
        output = f"/{ns}/sensor_measurements"
        for sensor in robot.get("sensors", []):
            if not sensor.get("enabled", True):
                continue
            kind = sensor.get("type")
            if kind not in ("camera", "lidar", "imu"):
                continue
            sid = identifier(sensor["id"])
            if sensor["parent-link"] != body_name:
                raise ValueError(f"{sid}: only root-link sensors are supported")
            config = pose(sensor)
            frame = f"{ns}/{sid}"
            if kind == "camera":
                captures = [c for c in sensor.get("capture-settings", [])
                            if c.get("capture-enabled", False)]
                if not captures:
                    continue
                if any(sensor.get("gimbal", {}).get(k, False)
                       for k in ("lock-roll", "lock-pitch", "lock-yaw")):
                    raise ValueError(f"{sid}: stabilized camera requires dynamic TF")
                frame += "_optical"
                for capture in captures:
                    image_type = capture["image-type"]
                    if image_type not in IMAGES:
                        raise ValueError(f"{sid}: unsupported image-type {image_type}")
                    native, common = IMAGES[image_type]
                    result.append(dict(kind="image", source=f"{source}/{sid}/{native}",
                        target=f"{output}/{sid}/{common}/image", frame=frame,
                        parent=f"{ns}/base_link", transform=extrinsic(config, optical=True),
                        capture=capture))
            elif kind == "imu":
                # Native IMU samples vehicle kinematics and ignores sensor pose.
                if any(config.values()):
                    raise ValueError(f"{sid}: native IMU extrinsics are not implemented upstream")
                result.append(dict(kind="imu", source=f"{source}/{sid}/imu",
                    target=f"{output}/{sid}/imu", frame=frame,
                    parent=f"{ns}/base_link", transform=extrinsic(config)))
            else:
                result.append(dict(kind="lidar", source=f"{source}/{sid}/lidar",
                    target=f"{output}/{sid}/points", frame=frame,
                    parent=f"{ns}/base_link", transform=extrinsic(config)))
    if not result:
        raise ValueError("no supported enabled sensors")
    return result
