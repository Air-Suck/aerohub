"""Config-derived artifacts for the mixed RoboPi/PC Swarm-Formation entry.

Pure functions only: no Docker, RoboPi, ROS or AirSim access, so the generated
vehicles, ports, containers, task and role placement can be unit tested.
"""
from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any

import yaml

from deployment.swarm.topology import ConfigError, render_airsim_settings, resolve_topology


ROOT = Path(__file__).resolve().parents[4]
APP = Path(__file__).resolve().parent
PREFIX = "aerohub-swarm-field-"
NATIVE_PARAMETERS = (
    ROOT / "src/tool/navigation/planner/swarm_formation/ros2_adapter/native/plan_manage/config/humble.yaml"
)
# seven_planners.launch.py, setDesiredFormation and the Swarm adapter all reject other sizes.
MIN_VEHICLES = 3
MAX_VEHICLES = 7
# sim_startup, the clock/sensor containers and the board RPC relay use this AirSim RPC port.
RPC_PORT = 42471
BOARD_RPC_RELAY_PORT = 42473
# The centralized proxy and the adapter that queries it both use loopback only.
GOAL_OFFSET_NED_M = (6.0, 0.0)
GOAL_DOWN_NED_M = -3.0


def validate_demo_topology(config: dict[str, Any], topology: dict[str, Any]) -> None:
    count = topology["vehicle_count"]
    if not MIN_VEHICLES <= count <= MAX_VEHICLES:
        raise ConfigError(
            f"Swarm-Formation hybrid entry supports {MIN_VEHICLES}..{MAX_VEHICLES} vehicles, got {count}"
        )
    if topology["coordination_mode"] != "centralized":
        raise ConfigError("Swarm-Formation hybrid entry requires centralized coordination")
    for vehicle in topology["vehicles"]:
        index = vehicle["index"]
        if vehicle["name"] != f"drone{index}" or vehicle["airsim_name"] != f"Drone{index}":
            raise ConfigError(
                "native launch, flight mission and audit address vehicles as "
                "drone<i>/Drone<i>; set name_prefix=drone and airsim_name_prefix=Drone"
            )
    if int(config["airsim"].get("rpc_port", RPC_PORT)) != RPC_PORT:
        raise ConfigError(f"airsim.rpc_port must be {RPC_PORT} for this entry")


def formation_offsets_enu(
    count: int,
    planner_parameters: dict[str, Any],
    native_parameters: dict[str, Any],
) -> tuple[float, list[tuple[float, float, float]]]:
    """Scaled native goal offsets: end_pt = center + swarm_scale * relative_pos_i (ENU)."""
    parameters = dict(native_parameters)
    parameters.update(planner_parameters)
    scale = float(parameters.get("global_goal.swarm_scale", 1.0))
    offsets = []
    for index in range(count):
        key = f"global_goal.relative_pos_{index}"
        try:
            offsets.append(tuple(scale * float(parameters[f"{key}.{axis}"]) for axis in "xyz"))
        except KeyError as exc:
            raise ConfigError(f"native Swarm parameters lack {exc.args[0]}") from exc
    return scale, offsets


def formation_targets(
    vehicles: list[dict[str, Any]],
    offsets_enu: list[tuple[float, float, float]],
) -> dict[str, dict[str, float]]:
    """Fleet-NED targets in the native formation shape, translated ahead of the start centroid.

    initial_xy_m is AirSim X/Y, i.e. fleet north/east. Native offsets are ENU
    (x east, y north, z up), so north uses offset[1] and east uses offset[0].
    """
    count = float(len(vehicles))
    start_north = sum(vehicle["initial_xy_m"][0] for vehicle in vehicles) / count
    start_east = sum(vehicle["initial_xy_m"][1] for vehicle in vehicles) / count
    mean_east = sum(offset[0] for offset in offsets_enu) / count
    mean_north = sum(offset[1] for offset in offsets_enu) / count
    center_north = start_north + GOAL_OFFSET_NED_M[0] - mean_north
    center_east = start_east + GOAL_OFFSET_NED_M[1] - mean_east
    return {
        vehicle["name"]: {
            "x": center_north + offset[1],
            "y": center_east + offset[0],
            "z": GOAL_DOWN_NED_M - offset[2],
        }
        for vehicle, offset in zip(vehicles, offsets_enu)
    }


def container_names(topology: dict[str, Any]) -> dict[str, list[str]]:
    count = topology["vehicle_count"]
    return {
        "services": [PREFIX + "clock", PREFIX + "audit", PREFIX + "ground"],
        "vehicles": [PREFIX + f"vehicle{index}" for index in range(count)],
        "sensors": [PREFIX + f"sensor{index}" for index in range(count)],
        "compute": [PREFIX + f"compute{vehicle['index']}" for vehicle in topology["pc_vehicles"]],
    }


def role_placement(topology: dict[str, Any], scenario: dict[str, Any]) -> dict[str, Any]:
    board = topology["board_vehicle"]["name"]
    center = scenario["center_vehicle"]
    return {
        "board_uav_node": {
            "vehicle": board,
            "placement": "robopi",
            "processes": ["as2", "navigation_inputs", "swarm_input_adapter", "native_planner"],
        },
        "pc_uav_nodes": [
            {
                "vehicle": vehicle["name"],
                "placement": "pc",
                "supervisor": "one ComputeWorker thread",
                "container": PREFIX + f"compute{vehicle['index']}",
            }
            for vehicle in topology["pc_vehicles"]
        ],
        "center_coordinator": {
            "vehicle": center,
            "process": "ego_planner ros2_proxy.py",
            "placement": "robopi" if center == board else "pc",
            "bind": "0.0.0.0:28501",
        },
        "task_client": {
            "processes": ["mission.py", "planner_framework_node", "swarm_formation_adapter"],
            "placement": "pc",
            "container": PREFIX + "ground",
        },
        "pc_observers": [
            {"process": "audit.py", "placement": "pc", "scope": "read-only DDS audit, full flight only"},
        ],
        "ground_station_on_pc": True,
        "known_gaps": [],
    }



def build_plan(
    config: dict[str, Any],
    scenario_name: str,
    task_template: dict[str, Any],
    planner_parameters: dict[str, Any],
    native_parameters: dict[str, Any],
) -> dict[str, Any]:
    topology = resolve_topology(config)
    validate_demo_topology(config, topology)
    scenario = next(
        (item for item in topology["scenarios"] if item["name"] == scenario_name), None
    )
    if scenario is None:
        raise ConfigError(f"unknown coordination scenario: {scenario_name}")
    vehicles = topology["vehicles"]
    count = topology["vehicle_count"]
    scale, offsets = formation_offsets_enu(count, planner_parameters, native_parameters)
    task_scale = float(task_template["planner"]["parameters"].get("formation_scale", 1.0))
    if abs(task_scale - scale) > 1e-9:
        raise ConfigError(
            f"task formation_scale {task_scale} differs from native swarm_scale {scale}"
        )
    task = copy.deepcopy(task_template)
    task["task"]["name"] = f"swarm_nh_open_field_{count}"
    task["vehicles"] = [vehicle["name"] for vehicle in vehicles]
    task["targets"] = formation_targets(vehicles, offsets)
    fleet = {
        "vehicles": {
            vehicle["name"]: {
                "fmu_prefix": "/" + vehicle["name"],
                "target_system_id": vehicle["px4_system_id"],
            }
            for vehicle in vehicles
        }
    }
    board = topology["board_vehicle"]
    return {
        "topology": topology,
        "scenario": scenario,
        "settings": render_airsim_settings(config),
        "fleet": fleet,
        "board_fleet": {"vehicles": {board["name"]: fleet["vehicles"][board["name"]]}},
        "deployment": {
            vehicle["name"]: {
                # AirSim X/Y are already fleet north/east; spawn height is the ground plane.
                "px4_origin_in_fleet_ned": [
                    float(vehicle["initial_xy_m"][0]), float(vehicle["initial_xy_m"][1]), 0.0,
                ],
                "namespace": "/" + vehicle["name"],
                "topics": {
                    "pose": "/" + vehicle["name"] + "/swarm/local_pose",
                    "trajectory_setpoint": "/" + vehicle["name"] + "/planner/setpoint",
                },
            }
            for vehicle in vehicles
        },
        "task": task,
        "sim_vehicles": [
            {
                key: vehicle[key]
                for key in ("name", "airsim_name", "px4_instance", "px4_system_id", "xrce_agent_port")
            }
            for vehicle in vehicles
        ],
        "containers": container_names(topology),
        "roles": role_placement(topology, scenario),
    }


def write_plan(plan: dict[str, Any], out: Path, base_profile: dict[str, Any], camera: dict[str, Any]) -> dict[str, Path]:
    """Write the generated artifacts; mapping keys preserve vehicle index order."""
    paths = {
        "settings": out / "settings.json",
        "profile": out / "profile.json",
        "task": out / "task.yaml",
        "deployment": out / "deployment.yaml",
        "fleet": out / "fleet.yaml",
        "board_fleet": out / "board-fleet.yaml",
    }
    paths["settings"].write_text(json.dumps(plan["settings"], indent=2) + "\n")
    profile = dict(base_profile, settings=str(paths["settings"]), overhead_view=camera)
    paths["profile"].write_text(json.dumps(profile, indent=2) + "\n")
    for key in ("task", "deployment", "fleet", "board_fleet"):
        paths[key].write_text(yaml.safe_dump(plan[key], sort_keys=False))
    return paths
