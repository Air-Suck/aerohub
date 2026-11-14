"""Strict local planner registry and resolved-task generation."""

from __future__ import annotations

import math
import uuid
from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


class ConfigError(ValueError):
    pass


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ConfigError(f"cannot load {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ConfigError(f"{path} must contain a mapping")
    return value


def recursive_merge(defaults: dict[str, Any], overrides: dict[str, Any]) -> dict[str, Any]:
    """Merge mappings recursively; arrays and scalar values replace defaults."""
    unknown = set(overrides) - set(defaults)
    if unknown:
        raise ConfigError(f"unknown parameter(s): {', '.join(sorted(unknown))}")
    result = deepcopy(defaults)
    for key, value in overrides.items():
        if isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = recursive_merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def _expect_keys(value: dict[str, Any], required: set[str], allowed: set[str], where: str) -> None:
    missing = required - set(value)
    unknown = set(value) - allowed
    if missing:
        raise ConfigError(f"{where} missing: {', '.join(sorted(missing))}")
    if unknown:
        raise ConfigError(f"{where} unknown field(s): {', '.join(sorted(unknown))}")


def _finite_number(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ConfigError(f"{where} must be a finite number")
    return float(value)


def _validate_parameters(value: Any, schema: dict[str, Any], where: str = "parameters") -> None:
    kind = schema.get("type")
    if kind == "object":
        if not isinstance(value, dict):
            raise ConfigError(f"{where} must be a mapping")
        properties = schema.get("properties", {})
        if schema.get("additionalProperties", True) is False:
            unknown = set(value) - set(properties)
            if unknown:
                raise ConfigError(f"{where} unknown field(s): {', '.join(sorted(unknown))}")
        for name in schema.get("required", []):
            if name not in value:
                raise ConfigError(f"{where}.{name} is required")
        for name, item in value.items():
            if name in properties:
                _validate_parameters(item, properties[name], f"{where}.{name}")
        return
    if kind == "array":
        if not isinstance(value, list):
            raise ConfigError(f"{where} must be an array")
        for index, item in enumerate(value):
            _validate_parameters(item, schema.get("items", {}), f"{where}[{index}]")
        return
    if kind == "number":
        numeric = _finite_number(value, where)
        if "minimum" in schema and numeric < schema["minimum"]:
            raise ConfigError(f"{where} is below minimum")
        if "exclusiveMinimum" in schema and numeric <= schema["exclusiveMinimum"]:
            raise ConfigError(f"{where} must exceed minimum")
    elif kind == "integer" and (isinstance(value, bool) or not isinstance(value, int)):
        raise ConfigError(f"{where} must be an integer")
    elif kind == "string" and not isinstance(value, str):
        raise ConfigError(f"{where} must be a string")
    elif kind == "boolean" and not isinstance(value, bool):
        raise ConfigError(f"{where} must be a boolean")
    if "enum" in schema and value not in schema["enum"]:
        raise ConfigError(f"{where} is not an allowed value")


class PlannerRegistry:
    def __init__(self, root: str | Path):
        self.root = Path(root)

    def resolve(self, task: dict[str, Any], deployment: dict[str, Any]) -> dict[str, Any]:
        _expect_keys(task, {"task", "planner", "vehicles", "targets", "completion"},
                     {"task", "planner", "vehicles", "targets", "completion"}, "task config")
        planner = task["planner"]
        _expect_keys(planner, {"id", "version"},
                     {"id", "version", "parameters"}, "planner")
        manifest_path = self.root / planner["id"] / planner["version"] / "manifest.yaml"
        manifest = _load_yaml(manifest_path)
        self._validate_manifest(manifest, planner["id"], planner["version"])
        base = manifest_path.parent
        defaults = _load_yaml(base / manifest["parameters"]["defaults"])
        parameter_schema = _load_yaml(base / manifest["parameters"]["schema"])
        parameters = recursive_merge(defaults, planner.get("parameters", {}))
        _validate_parameters(parameters, parameter_schema)
        self._validate_task(task, manifest, deployment)
        return {
            "task": {"name": task["task"]["name"], "task_id": str(uuid.uuid4())},
            "planner": {
                "id": manifest["id"], "version": manifest["version"],
                "manifest": str(manifest_path.resolve()),
                "runtime": deepcopy(manifest["runtime"]),
                "parameters": parameters,
            },
            "vehicles": deepcopy(task["vehicles"]),
            "targets": deepcopy(task["targets"]),
            "completion": deepcopy(task["completion"]),
            "coordinates": {"frame": "LOCAL_NED", "body_frame": "FRD"},
            "deployment": deepcopy(deployment),
        }

    @staticmethod
    def _validate_manifest(manifest: dict[str, Any], planner_id: str, version: str) -> None:
        required = {"id", "version", "runtime", "parameters", "capabilities", "output", "resources", "platforms"}
        _expect_keys(manifest, required, required, "manifest")
        if manifest["id"] != planner_id or str(manifest["version"]) != str(version):
            raise ConfigError("manifest identity does not match registry path")
        runtime = manifest["runtime"]
        _expect_keys(runtime, {"image", "adapter", "command"}, {"image", "adapter", "command"}, "manifest.runtime")
        if any(not isinstance(runtime[key], str) or not runtime[key] for key in ("image", "adapter")):
            raise ConfigError("manifest runtime image and adapter must be non-empty strings")
        if not isinstance(runtime["command"], list) or not runtime["command"] or any(
            not isinstance(item, str) or not item for item in runtime["command"]
        ):
            raise ConfigError("manifest runtime command must be a non-empty string array")
        parameters = manifest["parameters"]
        _expect_keys(parameters, {"defaults", "schema"}, {"defaults", "schema"}, "manifest.parameters")
        if any(not isinstance(parameters[key], str) or not parameters[key] for key in parameters):
            raise ConfigError("manifest parameter files must be non-empty strings")
        capabilities = manifest["capabilities"]
        _expect_keys(
            capabilities,
            {"planning_mode", "min_vehicles", "max_vehicles", "required_inputs"},
            {"planning_mode", "min_vehicles", "max_vehicles", "required_inputs"},
            "manifest.capabilities",
        )
        if (
            not isinstance(capabilities["planning_mode"], str)
            or isinstance(capabilities["min_vehicles"], bool)
            or not isinstance(capabilities["min_vehicles"], int)
            or isinstance(capabilities["max_vehicles"], bool)
            or not isinstance(capabilities["max_vehicles"], int)
            or capabilities["min_vehicles"] < 1
            or capabilities["max_vehicles"] < capabilities["min_vehicles"]
        ):
            raise ConfigError("manifest capabilities are invalid")
        inputs = capabilities["required_inputs"]
        if not isinstance(inputs, list) or any(not isinstance(item, str) or not item for item in inputs):
            raise ConfigError("manifest required_inputs must be a string array")
        if manifest["output"] != {"type": "TimedTrajectorySegment", "interface_version": 1}:
            raise ConfigError("unsupported planner output")
        resources = manifest["resources"]
        _expect_keys(resources, {"gpu_required"}, {"cpu", "memory_mb", "gpu_required"}, "manifest.resources")
        if not isinstance(resources["gpu_required"], bool):
            raise ConfigError("manifest.resources.gpu_required must be boolean")
        for key in ("cpu", "memory_mb"):
            if key in resources and _finite_number(resources[key], f"manifest.resources.{key}") <= 0:
                raise ConfigError(f"manifest.resources.{key} must be positive")
        platforms = manifest["platforms"]
        if not isinstance(platforms, list) or not platforms or any(
            not isinstance(item, str) or not item for item in platforms
        ):
            raise ConfigError("manifest platforms must be a non-empty string array")

    @staticmethod
    def _validate_task(task: dict[str, Any], manifest: dict[str, Any], deployment: dict[str, Any]) -> None:
        _expect_keys(task["task"], {"name"}, {"name"}, "task")
        completion = task["completion"]
        _expect_keys(completion, {"position_tolerance", "hold_duration", "task_timeout"},
                     {"position_tolerance", "hold_duration", "task_timeout"}, "completion")
        for key in completion:
            if _finite_number(completion[key], f"completion.{key}") <= 0:
                raise ConfigError(f"completion.{key} must be positive")
        vehicles = task["vehicles"]
        if not isinstance(vehicles, list) or not vehicles or any(not isinstance(v, str) or not v for v in vehicles):
            raise ConfigError("vehicles must be a non-empty string array")
        if len(set(vehicles)) != len(vehicles):
            raise ConfigError("vehicles must not contain duplicates")
        capabilities = manifest["capabilities"]
        if not capabilities["min_vehicles"] <= len(vehicles) <= capabilities["max_vehicles"]:
            raise ConfigError("vehicle count is outside planner capability")
        targets = task["targets"]
        if not isinstance(targets, dict) or set(targets) != set(vehicles):
            raise ConfigError("targets must match vehicles exactly")
        for vehicle, target in targets.items():
            if not isinstance(target, dict) or set(target) != {"x", "y", "z"}:
                raise ConfigError(f"targets.{vehicle} must contain only x, y, z")
            for axis in ("x", "y", "z"):
                _finite_number(target[axis], f"targets.{vehicle}.{axis}")
        if set(deployment) != set(vehicles):
            raise ConfigError("deployment must match vehicles exactly")
        for vehicle, item in deployment.items():
            required = {"px4_origin_in_fleet_ned", "namespace", "topics"}
            if not isinstance(item, dict) or not required <= set(item):
                raise ConfigError(f"deployment.{vehicle} is incomplete")
            origin = item["px4_origin_in_fleet_ned"]
            if not isinstance(origin, list) or len(origin) != 3:
                raise ConfigError(f"deployment.{vehicle}.px4_origin_in_fleet_ned must have three values")
            for index, coordinate in enumerate(origin):
                _finite_number(coordinate, f"deployment.{vehicle}.px4_origin_in_fleet_ned[{index}]")
