#!/usr/bin/env python3
"""Prepare a PX4 sensor example using upstream config loading and sensor definitions."""
import argparse
import copy
import hashlib
import json
from pathlib import Path

from projectairsim.utils import load_scene_config_as_dict


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-config-path", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--lidar", action="store_true")
    args = parser.parse_args()
    source = args.sim_config_path.resolve()
    scene, paths = load_scene_config_as_dict("scene_px4_sitl.jsonc", str(source))
    robot = scene["actors"][0]["robot-config"]
    if args.lidar:
        sensors_scene, sensors_paths = load_scene_config_as_dict("scene_drone_sensors.jsonc", str(source))
        lidar = [copy.deepcopy(sensor) for sensor in
                 sensors_scene["actors"][0]["robot-config"]["sensors"]
                 if sensor.get("type") == "lidar" and sensor.get("enabled")]
        if not lidar:
            raise ValueError("upstream sensor scene has no enabled lidar")
        links = {link.get("name") for link in robot.get("links", [])}
        if any(sensor["parent-link"] not in links for sensor in lidar):
            raise ValueError("upstream lidar references an unknown PX4 robot link")
        robot["sensors"].extend(lidar)
        paths[1].extend(sensors_paths[1])
        paths[1].append(sensors_paths[0])
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "robot.json").write_text(json.dumps(robot, indent=2) + "\n")
    scene["actors"][0]["robot-config"] = "robot.json"
    (args.output / "scene.json").write_text(json.dumps(scene, indent=2) + "\n")
    inputs = {str(path): hashlib.sha256(Path(path).read_bytes()).hexdigest()
              for path in [paths[0], *paths[1]]}
    (args.output / "inputs.json").write_text(json.dumps(inputs, indent=2) + "\n")
    print(args.output / "scene.json")


if __name__ == "__main__":
    main()
