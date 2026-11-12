"""Translate the user YAML into the existing simulator manager profile."""
from copy import deepcopy
from pathlib import Path


def projectairsim_profile(config: dict, config_path: Path) -> dict:
    """Keep native bridge arguments and backend paths out of the user interface."""
    config = deepcopy(config)
    unknown = set(config) - {"backend", "render", "ros_domain_id", "projectairsim", "scene", "vehicles", "ports", "display"}
    if unknown:
        raise ValueError(f"unknown simulator settings: {sorted(unknown)}")
    installation = config["projectairsim"]
    from simulator.manager.resources import resolve_resource
    for field in ("source", "binary", "python", "ros_setup", "project"):
        if field not in installation and field + "_resource" in installation:
            installation[field] = str(resolve_resource(installation[field + "_resource"]))

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
    profile = {
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

    if "display" in config:
        display = {"enabled": True, "camera_enabled": False, "vehicle": vehicles[0]["name"],
                   "camera": "DownCamera", "camera_width": 320,
                   "speed_max_mps": 20, "port": 8991}
        if not isinstance(config["display"], dict) or set(config["display"]) - set(display):
            raise ValueError("unknown display settings")
        display.update(config["display"])
        for field in ("enabled", "camera_enabled"):
            if not isinstance(display[field], bool):
                raise ValueError(f"display.{field} must be boolean")
        for field in ("vehicle", "camera"):
            if not isinstance(display[field], str) or not display[field]:
                raise ValueError(f"display.{field} must be a nonempty name")
        if (type(display["port"]) is not int or not 1 <= display["port"] <= 65535
                or display["port"] in profile["ports"].values()):
            raise ValueError("display.port must be a distinct valid port")
        if (type(display["camera_width"]) not in (int, float)
                or not 160 <= display["camera_width"] <= 640):
            raise ValueError("display.camera_width must be between 160 and 640")
        if (type(display["speed_max_mps"]) not in (int, float)
                or not 1 <= display["speed_max_mps"] <= 100):
            raise ValueError("display.speed_max_mps must be between 1 and 100")
        profile["display"] = display
        profile["editor_project"] = path(installation.get("project", str(source / "unreal/Blocks/Blocks.uproject")))
    return profile
