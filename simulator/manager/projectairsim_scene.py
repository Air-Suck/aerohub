#!/usr/bin/env python3
"""Load one ProjectAirSim scene, then leave it owned by the simulator host."""
import argparse
import json
import sys
import socket
import time
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--client-path", type=Path, required=True)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--address", default="127.0.0.1")
    parser.add_argument("--topic-port", type=int, default=8989)
    parser.add_argument("--service-port", type=int, default=8990)
    parser.add_argument("--runtime", type=Path)
    parser.add_argument("--profile", type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.client_path))
    from projectairsim import ProjectAirSimClient, World
    profile = json.loads(args.profile.read_text()) if args.profile else {}
    lidar_template = None
    if any(s.get("type") == "lidar" for sensors in profile.get("sensor_options", {}).values()
           for s in sensors.values()):
        from projectairsim.utils import load_scene_config_as_dict
        template_path = Path(profile["lidar_template"])
        template_scene, _ = load_scene_config_as_dict(template_path.name, str(template_path.parent))
        lidar_template = next(s for s in template_scene["actors"][0]["robot-config"]["sensors"]
                              if s["type"] == "lidar" and s.get("enabled", True))

    class ManagedWorld(World):
        def load_scene(self, scene_config_dict, delay_after_load_sec=0):
            if profile.get("sensor_options"):
                sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
                from simulator.projectairsim.scene_config import configure_sensors
                scene_config_dict = configure_sensors(scene_config_dict, profile["sensor_options"], lidar_template)
            if args.runtime:
                from projectairsim_config import configure_scene
                scene_config_dict = configure_scene(
                    scene_config_dict, json.loads(args.runtime.read_text()))
                (args.runtime.parent / "resolved_scene.json").write_text(
                    json.dumps(scene_config_dict, indent=2) + "\n")
            return super().load_scene(scene_config_dict, delay_after_load_sec)

    deadline = time.monotonic() + args.timeout
    for port in (args.topic_port, args.service_port):
        while True:
            try:
                with socket.create_connection((args.address, port), timeout=0.5):
                    break
            except OSError:
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"ProjectAirSim port {port} did not become ready")
                time.sleep(0.1)
    client = ProjectAirSimClient(args.address, args.topic_port, args.service_port)
    try:
        client.connect()
        ManagedWorld(
            client,
            args.scene.name,
            delay_after_load_sec=0,
            sim_config_path=str(args.scene.parent),
        )
    finally:
        client.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
