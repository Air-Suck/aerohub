"""Translate the user YAML into the existing simulator manager profile."""
from copy import deepcopy
from pathlib import Path


def projectairsim_profile(config: dict, config_path: Path) -> dict:
    """Keep native bridge arguments and backend paths out of the user interface."""
    config = deepcopy(config)
    unknown = set(config) - {"backend", "render", "ros_domain_id", "projectairsim", "scene", "vehicles", "ports"}
    if unknown:
        raise ValueError(f"unknown simulator settings: {sorted(unknown)}")
    installation = config["projectairsim"]

    def path(value):
        value = Path(value).expanduser()
        return str(value if value.is_absolute() else config_path.parent / value)

    source = Path(path(installation["source"]))
    scene = config.get("scene", "scene_px4_sitl.jsonc")
    scene_path = Path(scene).expanduser()
    if not scene_path.is_absolute():
        scene_path = source / "client/python/example_user_scripts/sim_config" / scene
    vehicles = config["vehicles"]
    for index, vehicle in enumerate(vehicles):
        vehicle.setdefault("namespace", f"drone{index}")
        vehicle.setdefault("px4", {})
        defaults = {"tcp_port": 4560 + index, "control_local_port": 14540 + index,
                    "control_remote_port": 14580 + index, "lockstep": True}
        vehicle["px4"] = defaults | vehicle["px4"]
    args = ["Blocks", "-vulkan", "-nosound"]
    render = config.get("render", "windowed")
    if render not in ("windowed", "offscreen"):
        raise ValueError("render must be windowed or offscreen")
    if render == "offscreen":
        args.append("-RenderOffscreen")
    return {
        "backend": "projectairsim", "binary": path(installation["binary"]),
        "client_path": str(source / "client/python/projectairsim/src"),
        "client_python": path(installation["python"]), "scene_config": str(scene_path),
        "sensor_options": {v["name"]: v.pop("sensors", {}) for v in vehicles},
        "lidar_template": str(source / "client/python/example_user_scripts/sim_config/scene_drone_sensors.jsonc"),
        "vehicles": vehicles, "ports": config.get("ports", {"topics": 8989, "services": 8990}),
        "args": args,
        "adapter": {
            "command": [str(Path(__file__).with_name("projectairsim_adapter.sh"))],
            "setup": path(installation["ros_setup"]), "sensor_contract": True,
            "environment": {
                "PROJECTAIRSIM_ROS_IMAGE": installation.get("ros_image", "onboard_computer_dependencies:humble-px4-1.18"),
                "ROS_DOMAIN_ID": str(config.get("ros_domain_id", 0)),
            },
            "ros_args": ["-p", f"vehicle_name:={vehicles[0]['name']}",
                         "-p", "ros_topic_root:=/ProjectAirsim", "-p", "image_qos_depth:=1"],
        },
    }
