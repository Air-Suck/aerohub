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
    args = parser.parse_args()
    sys.path.insert(0, str(args.client_path))
    from projectairsim import ProjectAirSimClient, World

    class ManagedWorld(World):
        def load_scene(self, scene_config_dict, delay_after_load_sec=0):
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
