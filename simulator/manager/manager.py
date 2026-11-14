#!/usr/bin/env python3
"""Manage one AirSim or ProjectAirSim host from a common profile."""
from __future__ import annotations

import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from simulator.manager.config import load_profile, runtime_manifest


def _ticks(pid: int) -> str:
    fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
    if fields[0] == "Z":
        raise ProcessLookupError(pid)
    return fields[19]


def _alive(state: dict) -> bool:
    if state.get("owns_host") is False:
        if state.get("phase") == "stopped":
            return False
        try:
            for port in state["profile"]["ports"].values():
                with socket.create_connection(
                    (state["profile"].get("address", "127.0.0.1"), port), timeout=0.5
                ):
                    pass
            return True
        except (OSError, KeyError):
            return False
    try:
        return (
            _ticks(int(state["pid"])) == state["start_ticks"]
            and Path(f"/proc/{state['pid']}/exe").resolve() == Path(state["binary"]).resolve()
        )
    except (OSError, KeyError, ProcessLookupError, ValueError):
        return False


def _process_alive(process: dict) -> bool:
    try:
        return _ticks(int(process["pid"])) == process["start_ticks"]
    except (OSError, KeyError, ProcessLookupError, ValueError):
        return False


def _write(run_dir: Path, state: dict) -> None:
    temporary = run_dir / "state.json.tmp"
    temporary.write_text(json.dumps(state, indent=2) + "\n")
    temporary.replace(run_dir / "state.json")


def _ports_free(ports: list[int]) -> None:
    sockets = []
    try:
        for port in ports:
            sock = socket.socket()
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", port))
            sockets.append(sock)
    finally:
        for sock in sockets:
            sock.close()


def _terminate(state: dict, run_dir: Path) -> None:
    for process in reversed(state.get("auxiliaries", [])):
        if _process_alive(process):
            os.killpg(int(process["pid"]), signal.SIGTERM)
            deadline = time.monotonic() + 5
            while _process_alive(process) and time.monotonic() < deadline:
                time.sleep(0.1)
            if _process_alive(process):
                os.killpg(int(process["pid"]), signal.SIGKILL)
    if state.get("owns_host", True) and _alive(state):
        os.killpg(int(state["pid"]), signal.SIGTERM)
        deadline = time.monotonic() + 8
        while _alive(state) and time.monotonic() < deadline:
            time.sleep(0.1)
        if _alive(state):
            os.killpg(int(state["pid"]), signal.SIGKILL)
    state["phase"] = "stopped"
    _write(run_dir, state)


def _airsim_command(args, action: str) -> list[str]:
    command = [
        sys.executable,
        str(ROOT / "simulator/airsim/manager/manager.py"),
        action,
        "--run-dir",
        str(args.run_dir),
    ]
    if args.profile:
        command += ["--profile", str(args.profile)]
    if args.binary:
        command += ["--binary", args.binary]
    if action == "start":
        command += ["--timeout", str(args.timeout)]
    return command


def _start_projectairsim(profile: dict, run_dir: Path, timeout: float, attach: bool = False) -> dict:
    child = None
    state = {
        "backend": "projectairsim", "owns_host": not attach,
        "profile": profile, "phase": "starting", "auxiliaries": [],
    }
    from simulator.manager.display import write_display_config
    display_path = write_display_config(profile, run_dir, attach=attach)
    if not attach:
        binary = Path(profile["binary"])
        if not binary.is_file() or not os.access(binary, os.X_OK):
            raise ValueError(f"ProjectAirSim host is not executable: {binary}")
        ports = list(profile["ports"].values())
        if profile.get("display", {}).get("enabled", False):
            ports.append(profile["display"]["port"])
        _ports_free(ports)
        command = [str(binary), *profile["args"],
                   f"-topicsport={profile['ports']['topics']}",
                   f"-servicesport={profile['ports']['services']}"]
        log_path = run_dir / "projectairsim.log"
        with log_path.open("w") as log:
            environment = dict(os.environ)
            if display_path:
                environment["AEROHUB_DISPLAY_CONFIG"] = str(display_path)
            child = subprocess.Popen(
                command, cwd=binary.parent, env=environment, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        state.update(pid=child.pid, start_ticks=_ticks(child.pid), binary=str(binary),
                     command=command, log=str(log_path))
    (run_dir / "runtime.json").write_text(json.dumps(runtime_manifest(profile), indent=2) + "\n")
    (run_dir / "profile.json").write_text(json.dumps(profile, indent=2) + "\n")
    _write(run_dir, state)
    loader = [
        profile["client_python"],
        str(Path(__file__).with_name("projectairsim_scene.py")),
        "--client-path",
        profile["client_path"],
        "--scene",
        profile["scene_config"],
        "--timeout",
        str(timeout),
        "--address", profile.get("address", "127.0.0.1"),
        "--topic-port", str(profile["ports"]["topics"]),
        "--service-port", str(profile["ports"]["services"]),
        "--runtime", str(run_dir / "runtime.json"),
        "--profile", str(run_dir / "profile.json"),
    ]
    try:
        result = subprocess.run(loader, text=True, capture_output=True, timeout=timeout + 5)
        if result.returncode:
            raise RuntimeError(
                "ProjectAirSim scene load failed: " + (result.stderr.strip() or result.stdout.strip())
            )
        if child is not None and child.poll() is not None:
            raise RuntimeError(f"ProjectAirSim exited; see {log_path}")
        state["phase"] = "ready"
        state["scene_loader"] = loader
        state["auxiliaries"] = []
        adapter = profile.get("adapter")
        if adapter:
            environment = dict(os.environ)
            environment.update(adapter.get("environment", {}))
            if "setup" in adapter:
                environment["PROJECTAIRSIM_ROS_SETUP"] = adapter["setup"]
            environment["PROJECTAIRSIM_ADDRESS"] = profile.get("address", "127.0.0.1")
            environment["PROJECTAIRSIM_TOPIC_PORT"] = str(profile["ports"]["topics"])
            environment["PROJECTAIRSIM_SERVICE_PORT"] = str(profile["ports"]["services"])
            environment["PROJECTAIRSIM_ROS_ARGS"] = json.dumps(adapter.get("ros_args", []))
            if adapter.get("sensor_contract", False):
                from simulator.projectairsim.sensor_contract import streams
                topic_root = "/ProjectAirsim"
                for value in adapter.get("ros_args", []):
                    if value.startswith("ros_topic_root:="):
                        topic_root = value.split(":=", 1)[1]
                inventory = streams(json.loads((run_dir / "resolved_scene.json").read_text()),
                                    runtime_manifest(profile), topic_root)
                (run_dir / "sensor-contract.json").write_text(json.dumps(inventory, indent=2) + "\n")
                environment["PROJECTAIRSIM_SENSOR_RUNTIME"] = str(run_dir)
                environment["PROJECTAIRSIM_SENSOR_TOPIC_ROOT"] = topic_root
            adapter_log = run_dir / "adapter.log"
            with adapter_log.open("w") as output:
                process = subprocess.Popen(
                    adapter["command"], env=environment, stdout=output,
                    stderr=subprocess.STDOUT, start_new_session=True,
                )
            state["auxiliaries"].append({
                "role": "adapter", "pid": process.pid,
                "start_ticks": _ticks(process.pid), "command": adapter["command"],
                "log": str(adapter_log),
            })
            _write(run_dir, state)
            time.sleep(1)
            if process.poll() is not None:
                raise RuntimeError(f"simulator adapter exited; see {adapter_log}")
        if profile.get("display", {}).get("enabled", False):
            from simulator.manager.display import request_display
            deadline = time.monotonic() + timeout
            while True:
                try:
                    request_display(profile, "status")
                    break
                except (OSError, ValueError):
                    if time.monotonic() >= deadline:
                        raise RuntimeError("HUD HTTP interface not ready; inspect projectairsim.log")
                    time.sleep(.2)
            settings = profile["display"]
            command = [profile["client_python"],
                       str(Path(__file__).parents[1] / "projectairsim/display_telemetry.py"),
                       "--scene", str(run_dir / "resolved_scene.json"),
                       "--address", profile.get("address", "127.0.0.1"),
                       "--vehicle", settings["vehicle"],
                       "--topic-port", str(profile["ports"]["topics"]),
                       "--service-port", str(profile["ports"]["services"]),
                       "--display-port", str(settings["port"])]
            environment = dict(os.environ)
            environment["PYTHONPATH"] = profile["client_path"] + os.pathsep + environment.get("PYTHONPATH", "")
            telemetry_log = run_dir / "display-telemetry.log"
            with telemetry_log.open("w") as output:
                process = subprocess.Popen(command, env=environment, stdout=output,
                                           stderr=subprocess.STDOUT, start_new_session=True)
            state["auxiliaries"].append({"role": "display-telemetry", "pid": process.pid,
                "start_ticks": _ticks(process.pid), "command": command, "log": str(telemetry_log)})
        _write(run_dir, state)
        return state
    except BaseException:
        _terminate(state, run_dir)
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("check", "start", "attach", "status", "stop", "display"))
    parser.add_argument("--config", "--profile", dest="profile", type=Path)
    parser.add_argument("--binary")
    parser.add_argument("--event", choices=("status", "start", "finish", "reset"), default="status")
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--render-mode", choices=("windowed", "offscreen"))
    args = parser.parse_args()
    profile = load_profile(args.profile, args.binary) if args.profile else None
    if args.render_mode:
        os.environ["AIRSIM_RENDER_MODE"] = args.render_mode
        if profile and profile["backend"] == "projectairsim":
            profile["args"] = [arg for arg in profile["args"]
                               if arg.lower() != "-renderoffscreen"]
            if args.render_mode == "offscreen":
                profile["args"].append("-RenderOffscreen")
    backend = profile["backend"] if profile else None
    state_path = args.run_dir.expanduser().resolve() / "state.json"
    if backend is None and state_path.exists():
        backend = json.loads(state_path.read_text()).get("backend", "airsim")
    if args.action == "display":
        from simulator.manager.display import request_display
        if profile is None and state_path.exists():
            profile = json.loads(state_path.read_text())["profile"]
        if not profile or profile["backend"] != "projectairsim":
            raise ValueError("display requires a ProjectAirSim session or --config")
        print(json.dumps(request_display(profile, args.event), indent=2))
        return 0
    if args.action == "attach" and backend != "projectairsim":
        raise ValueError("attach requires a ProjectAirSim config")
    if backend in (None, "airsim"):
        return subprocess.run(_airsim_command(args, args.action)).returncode
    if args.timeout <= 0:
        raise ValueError("timeout must be positive")
    if args.action == "check":
        print(json.dumps(profile, indent=2))
        return 0
    run_dir = args.run_dir.expanduser().resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    with (run_dir / "manager.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        state = json.loads(state_path.read_text()) if state_path.exists() else {}
        if args.action == "status":
            state["alive"] = _alive(state)
            state["auxiliaries_alive"] = all(
                _process_alive(process) for process in state.get("auxiliaries", [])
            )
            print(json.dumps(state, indent=2))
            return 0 if (state["alive"] and state["auxiliaries_alive"]
                         and state.get("phase") == "ready") else 1
        if args.action == "stop":
            _terminate(state, run_dir)
            return 0
        if _alive(state) or any(_process_alive(p) for p in state.get("auxiliaries", [])):
            raise RuntimeError("this run directory has a live session; stop it before starting or attaching")
        print(json.dumps(_start_projectairsim(profile, run_dir, args.timeout, attach=args.action == "attach"), indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, KeyError, ImportError) as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1)
