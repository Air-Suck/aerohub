"""Load the small common simulator profile shared by all backends."""
from __future__ import annotations

import json
from pathlib import Path
import sys
from typing import Any


SUPPORTED_BACKENDS = {"airsim", "projectairsim"}


def _resolve(profile_path: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (profile_path.parent / path).resolve()


def load_profile(profile_path: Path, binary_override: str | None = None) -> dict[str, Any]:
    profile_path = profile_path.expanduser().resolve()
    if profile_path.suffix in (".yaml", ".yml"):
        import yaml
        profile = yaml.safe_load(profile_path.read_text())
    else:
        profile = json.loads(profile_path.read_text())
    if not isinstance(profile, dict):
        raise ValueError("simulator config must be a mapping")
    if "projectairsim" in profile:
        if profile.get("backend", "projectairsim") != "projectairsim":
            raise ValueError("projectairsim installation requires backend: projectairsim")
        from simulator.manager.yaml_config import projectairsim_profile
        profile = projectairsim_profile(profile, profile_path)
    backend = profile.get("backend", "airsim")
    if backend not in SUPPORTED_BACKENDS:
        raise ValueError(f"unsupported simulator backend: {backend!r}")
    profile["backend"] = backend
    profile["profile_path"] = str(profile_path)
    resource_keys = {
        "binary_resource": "binary", "scene_config_resource": "scene_config",
        "client_resource": "client_path", "client_python_resource": "client_python",
    }
    if backend == "projectairsim" and any(key in profile for key in resource_keys):
        from simulator.manager.resources import resolve_resource
        for resource_key, target_key in resource_keys.items():
            if resource_key in profile and target_key not in profile:
                profile[target_key] = str(resolve_resource(profile[resource_key]))
    if binary_override:
        profile["binary"] = binary_override
    for key in ("binary", "settings", "scene_config", "client_path", "client_python"):
        if key in profile:
            if not isinstance(profile[key], str) or not profile[key].strip():
                raise ValueError(f"{key} must be a nonempty path")
            if key == "client_python":
                # Resolving a venv interpreter symlink loses its pyvenv.cfg.
                path = Path(profile[key]).expanduser()
                profile[key] = str(path if path.is_absolute() else profile_path.parent / path)
            else:
                profile[key] = str(_resolve(profile_path, profile[key]))
    if backend == "projectairsim":
        missing = [key for key in ("binary", "scene_config", "client_path") if key not in profile]
        if missing:
            raise ValueError("ProjectAirSim profile is missing: " + ", ".join(missing))
        ports = profile.setdefault("ports", {"topics": 8989, "services": 8990})
        if not isinstance(ports, dict) or set(ports) != {"topics", "services"}:
            raise ValueError("ProjectAirSim ports require topics and services")
        for name, value in ports.items():
            value = int(value)
            if not 1 <= value <= 65535:
                raise ValueError(f"invalid ProjectAirSim {name} port")
            ports[name] = value
        if ports["topics"] == ports["services"]:
            raise ValueError("ProjectAirSim topic and service ports must differ")
        profile.setdefault("args", [])
        profile.setdefault("client_python", sys.executable)
        if not isinstance(profile["args"], list) or not all(
            isinstance(value, str) for value in profile["args"]
        ):
            raise ValueError("args must be a list of strings")
        for key in ("binary", "scene_config", "client_path", "client_python"):
            if not Path(profile[key]).exists():
                raise ValueError(f"ProjectAirSim {key} does not exist: {profile[key]}")
        adapter = profile.get("adapter")
        if adapter is not None:
            allowed = {"command", "environment", "setup", "setup_resource", "ros_args", "sensor_contract"}
            if not isinstance(adapter, dict) or set(adapter) - allowed:
                raise ValueError("adapter contains unknown fields")
            if not isinstance(adapter.get("sensor_contract", False), bool):
                raise ValueError("adapter.sensor_contract must be boolean")
            if "setup_resource" in adapter:
                adapter["setup"] = str(resolve_resource(adapter["setup_resource"]))
                adapter.setdefault("command", [
                    str(Path(__file__).with_name("projectairsim_adapter.sh")),
                ])
            if not isinstance(adapter.get("command"), list) or not adapter["command"] or not all(
                isinstance(value, str) and value for value in adapter["command"]
            ):
                raise ValueError("adapter requires command or setup_resource")
            if not isinstance(adapter.get("environment", {}), dict) or not all(
                isinstance(key, str) and isinstance(value, str)
                for key, value in adapter.get("environment", {}).items()
            ):
                raise ValueError("adapter.environment must map strings to strings")
            if not isinstance(adapter.get("ros_args", []), list) or not all(
                isinstance(value, str) for value in adapter.get("ros_args", [])
            ):
                raise ValueError("adapter.ros_args must be a string list")
        vehicles = profile.get("vehicles")
        if not isinstance(vehicles, list) or not vehicles:
            raise ValueError("ProjectAirSim profile requires a nonempty vehicles list")
        names = set()
        for vehicle in vehicles:
            if not isinstance(vehicle, dict) or not isinstance(vehicle.get("name"), str):
                raise ValueError("each ProjectAirSim vehicle requires a name")
            if vehicle["name"] in names:
                raise ValueError(f"duplicate vehicle name: {vehicle['name']}")
            names.add(vehicle["name"])
            px4 = vehicle.get("px4")
            if not isinstance(px4, dict):
                raise ValueError(f"{vehicle['name']}: px4 mapping is required")
            for key in ("tcp_port", "control_local_port", "control_remote_port"):
                value = int(px4[key])
                if not 1 <= value <= 65535:
                    raise ValueError(f"{vehicle['name']}: invalid {key}")
                px4[key] = value
    return profile


def runtime_manifest(profile: dict[str, Any], settings: dict | None = None) -> dict[str, Any]:
    """Return the backend-neutral vehicle contract consumed by flight_board."""
    if profile["backend"] == "projectairsim":
        vehicles = profile["vehicles"]
    else:
        if settings is None:
            settings = json.loads(Path(profile["settings"]).read_text())
        vehicles = []
        for index, (name, source) in enumerate(settings.get("Vehicles", {}).items()):
            vehicles.append({
                "name": name,
                "namespace": source.get("Namespace", f"drone{index}"),
                "px4": {
                    "tcp_port": int(source.get("TcpPort", 4560 + index)),
                    "control_local_port": int(source.get("ControlPortLocal", 14540 + index)),
                    "control_remote_port": int(source.get("ControlPortRemote", 14580 + index)),
                    "lockstep": bool(source.get("LockStep", False)),
                },
            })
    return {"schema_version": 1, "backend": profile["backend"], "vehicles": vehicles}
