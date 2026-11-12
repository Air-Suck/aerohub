#!/usr/bin/env python3
"""Validate and resolve a mixed RoboPi/PC swarm topology."""
from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
from typing import Any

import yaml

try:
    from .view import apply_overhead_view, camera_options
except ImportError:
    from view import apply_overhead_view, camera_options


class ConfigError(ValueError):
    pass


def _mapping(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ConfigError(f"{path} must be a mapping")
    return value


def _command(value: Any, path: str) -> list[str]:
    if not isinstance(value, list) or not value or any(
        not isinstance(item, str) or not item for item in value
    ):
        raise ConfigError(f"{path} must be a nonempty string list")
    return list(value)


def load_config(path: Path) -> dict[str, Any]:
    resolved = path.expanduser().resolve()
    data = yaml.safe_load(resolved.read_text())
    config = _mapping(data, "config")
    validate_config(config)
    config = copy.deepcopy(config)
    config["_config_path"] = str(resolved)
    return config


def validate_config(config: dict[str, Any]) -> None:
    if config.get("schema_version") != 1:
        raise ConfigError("schema_version must be 1")
    fleet = _mapping(config.get("fleet"), "fleet")
    count = fleet.get("vehicle_count")
    if not isinstance(count, int) or isinstance(count, bool) or count < 1:
        raise ConfigError("fleet.vehicle_count must be an integer >= 1")
    for key in ("name_prefix", "airsim_name_prefix"):
        if not isinstance(fleet.get(key), str) or not fleet[key]:
            raise ConfigError(f"fleet.{key} must be a nonempty string")
    board_index = fleet.get("board_vehicle_index")
    if not isinstance(board_index, int) or not 0 <= board_index < count:
        raise ConfigError("fleet.board_vehicle_index must select one configured vehicle")

    nodes = _mapping(config.get("nodes"), "nodes")
    board = _mapping(nodes.get("board"), "nodes.board")
    pc = _mapping(nodes.get("pc"), "nodes.pc")
    _command(board.get("launch_command"), "nodes.board.launch_command")
    _command(pc.get("launch_command"), "nodes.pc.launch_command")

    coordination = _mapping(config.get("coordination"), "coordination")
    mode = coordination.get("mode")
    if mode not in ("centralized", "decentralized"):
        raise ConfigError("coordination.mode must be centralized or decentralized")
    if mode == "centralized":
        cases = coordination.get("centralized_cases")
        required = {"board_center", "board_worker"}
        if not isinstance(cases, list) or set(cases) != required or len(cases) != 2:
            raise ConfigError(
                "centralized systems must test exactly board_center and board_worker"
            )
        if count < 2:
            raise ConfigError("board_worker requires at least one PC vehicle")

    ground = _mapping(config.get("ground_station"), "ground_station")
    if ground.get("placement") != "pc":
        raise ConfigError("ground_station.placement must be pc")
    if not isinstance(ground.get("required"), bool):
        raise ConfigError("ground_station.required must be boolean")
    if ground["required"]:
        _command(ground.get("launch_command"), "ground_station.launch_command")

    airsim = _mapping(config.get("airsim"), "airsim")
    if not isinstance(airsim.get("base_settings"), str) or not airsim["base_settings"]:
        raise ConfigError("airsim.base_settings must be a nonempty path")
    camera_options(_mapping(airsim.get("camera"), "airsim.camera"))
    ports = _mapping(airsim.get("ports"), "airsim.ports")
    for key in (
        "tcp_base",
        "control_local_base",
        "control_remote_base",
        "px4_instance_base",
        "px4_system_id_base",
        "xrce_agent_base",
    ):
        value = ports.get(key)
        if not isinstance(value, int) or value < 0:
            raise ConfigError(f"airsim.ports.{key} must be a nonnegative integer")
    for key in ("tcp_base", "control_local_base", "control_remote_base", "xrce_agent_base"):
        if ports[key] + count - 1 > 65535 or ports[key] < 1:
            raise ConfigError(f"airsim.ports.{key} range is invalid")

    validation = _mapping(config.get("validation"), "validation")
    duration = validation.get("smoke_duration_s")
    if not isinstance(duration, (int, float)) or isinstance(duration, bool) or duration <= 0:
        raise ConfigError("validation.smoke_duration_s must be positive")


def _formation_positions(count: int, spacing: float) -> list[tuple[float, float]]:
    if spacing <= 0:
        raise ConfigError("fleet.initial_spacing_m must be positive")
    if count == 1:
        return [(0.0, 0.0)]
    if count == 2:
        return [(-spacing / 2.0, 0.0), (spacing / 2.0, 0.0)]
    radius = spacing / (2.0 * math.sin(math.pi / count))
    return [
        (
            radius * math.cos(2.0 * math.pi * index / count),
            radius * math.sin(2.0 * math.pi * index / count),
        )
        for index in range(count)
    ]


def resolve_topology(config: dict[str, Any]) -> dict[str, Any]:
    validate_config(config)
    fleet = config["fleet"]
    count = fleet["vehicle_count"]
    board_index = fleet["board_vehicle_index"]
    ports = config["airsim"]["ports"]
    positions = _formation_positions(count, float(fleet.get("initial_spacing_m", 4.0)))
    vehicles = []
    for index in range(count):
        placement = "board" if index == board_index else "pc"
        vehicles.append(
            {
                "index": index,
                "name": f"{fleet['name_prefix']}{index}",
                "airsim_name": f"{fleet['airsim_name_prefix']}{index}",
                "placement": placement,
                "supervisor": "remote_launcher" if placement == "board" else "independent_thread",
                "initial_xy_m": list(positions[index]),
                "px4_instance": ports["px4_instance_base"] + index,
                "px4_system_id": ports["px4_system_id_base"] + index,
                "xrce_agent_port": ports["xrce_agent_base"] + index,
            }
        )
    board = vehicles[board_index]
    pc_vehicles = [vehicle for vehicle in vehicles if vehicle["placement"] == "pc"]
    if config["coordination"]["mode"] == "centralized":
        scenarios = [
            {
                "name": "board_center",
                "center_vehicle": board["name"],
                "board_role": "center",
            },
            {
                "name": "board_worker",
                "center_vehicle": pc_vehicles[0]["name"],
                "board_role": "worker",
            },
        ]
    else:
        scenarios = [
            {
                "name": "decentralized",
                "center_vehicle": None,
                "board_role": "peer",
            }
        ]
    return {
        "schema_version": 1,
        "vehicle_count": count,
        "board_vehicle": board,
        "pc_vehicles": pc_vehicles,
        "vehicles": vehicles,
        "coordination_mode": config["coordination"]["mode"],
        "scenarios": scenarios,
        "ground_station": {
            "required": config["ground_station"]["required"],
            "placement": "pc",
        },
        "invariants": {
            "board_vehicle_count": 1,
            "pc_vehicle_count": count - 1,
            "pc_supervisor": "one independent thread per vehicle",
        },
    }


def resolve_path(config: dict[str, Any], value: str) -> Path:
    path = Path(value).expanduser()
    if path.is_absolute():
        return path.resolve()
    source = Path(config.get("_config_path", ".")).resolve()
    return (source.parent / path).resolve()


def render_airsim_settings(config: dict[str, Any]) -> dict[str, Any]:
    topology = resolve_topology(config)
    base_path = resolve_path(config, config["airsim"]["base_settings"])
    settings = json.loads(base_path.read_text())
    base_vehicles = settings.get("Vehicles")
    if not isinstance(base_vehicles, dict) or not base_vehicles:
        raise ConfigError("base AirSim settings require a vehicle template")
    template = copy.deepcopy(next(iter(base_vehicles.values())))
    ports = config["airsim"]["ports"]
    vehicles = {}
    for vehicle in topology["vehicles"]:
        index = vehicle["index"]
        rendered = copy.deepcopy(template)
        rendered.update(
            {
                "VehicleType": "PX4Multirotor",
                "UseTcp": True,
                "LockStep": True,
                "TcpPort": ports["tcp_base"] + index,
                "ControlPortLocal": ports["control_local_base"] + index,
                "ControlPortRemote": ports["control_remote_base"] + index,
                "X": vehicle["initial_xy_m"][0],
                "Y": vehicle["initial_xy_m"][1],
                "Z": 0.0,
            }
        )
        vehicles[vehicle["airsim_name"]] = rendered
    settings["Vehicles"] = vehicles
    settings["ApiServerPort"] = int(config["airsim"].get("rpc_port", 41471))
    settings["ClockType"] = "SteppableClock"
    settings["SimMode"] = "Multirotor"
    settings["RpcEnabled"] = True
    return apply_overhead_view(settings, config["airsim"]["camera"])


def command_context(
    topology: dict[str, Any],
    scenario: dict[str, Any],
    vehicle: dict[str, Any] | None,
    output: Path,
) -> dict[str, str]:
    return {
        "vehicle": vehicle["name"] if vehicle else "",
        "vehicle_index": str(vehicle["index"]) if vehicle else "",
        "airsim_vehicle": vehicle["airsim_name"] if vehicle else "",
        "center_vehicle": scenario["center_vehicle"] or "",
        "scenario": scenario["name"],
        "vehicle_count": str(topology["vehicle_count"]),
        "output": str(output),
    }


def format_command(command: list[str], context: dict[str, str]) -> list[str]:
    try:
        return [part.format(**context) for part in command]
    except KeyError as exc:
        raise ConfigError(f"unknown command placeholder: {exc.args[0]}") from exc


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("validate", "plan", "render-settings"))
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    config = load_config(args.config)
    if args.action == "validate":
        result = {"valid": True, "config": str(args.config.resolve())}
    elif args.action == "plan":
        result = resolve_topology(config)
    else:
        result = render_airsim_settings(config)
    text = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text)
    else:
        print(text, end="")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ConfigError, OSError, json.JSONDecodeError, yaml.YAMLError) as exc:
        raise SystemExit(str(exc))
