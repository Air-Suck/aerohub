#!/usr/bin/env python3
"""Run one independent onboard-computer service per UAV, optionally replacing one with a board."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from typing import Any

try:
    from .topology import (
        ConfigError,
        command_context,
        format_command,
        load_config,
        resolve_topology,
    )
except ImportError:
    from topology import ConfigError, command_context, format_command, load_config, resolve_topology


class ServiceThread(threading.Thread):
    def __init__(
        self,
        *,
        identity: str,
        placement: str,
        command: list[str],
        environment: dict[str, str],
        log_path: Path,
        stop_event: threading.Event,
    ):
        super().__init__(name=f"swarm-{identity}", daemon=False)
        self.identity = identity
        self.placement = placement
        self.command = command
        self.environment = environment
        self.log_path = log_path
        self.stop_event = stop_event
        self.ready = threading.Event()
        self.process: subprocess.Popen[str] | None = None
        self.thread_id: int | None = None
        self.error: str | None = None
        self.exited_before_stop = False

    def run(self) -> None:
        self.thread_id = threading.get_ident()
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            with self.log_path.open("w") as log:
                self.process = subprocess.Popen(
                    self.command,
                    env=self.environment,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    text=True,
                    start_new_session=True,
                )
                self.ready.set()
                while not self.stop_event.wait(0.05):
                    code = self.process.poll()
                    if code is not None:
                        self.exited_before_stop = True
                        self.error = f"process exited early with code {code}"
                        return
                self._terminate()
        except BaseException as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self.ready.set()

    def _terminate(self) -> None:
        if self.process is None or self.process.poll() is not None:
            return
        os.killpg(self.process.pid, signal.SIGINT)
        try:
            self.process.wait(timeout=3)
            return
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGTERM)
        try:
            self.process.wait(timeout=2)
            return
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGKILL)
            self.process.wait(timeout=2)

    def record(self) -> dict[str, Any]:
        return {
            "identity": self.identity,
            "placement": self.placement,
            "supervisor_thread_name": self.name,
            "supervisor_thread_id": self.thread_id,
            "pid": self.process.pid if self.process else None,
            "command": self.command,
            "returncode": self.process.poll() if self.process else None,
            "exited_before_stop": self.exited_before_stop,
            "error": self.error,
            "log": str(self.log_path),
        }


def _environment(
    base: dict[str, str],
    context: dict[str, str],
    placement: str,
    role: str,
) -> dict[str, str]:
    result = dict(base)
    result.update(
        {
            "AEROHUB_SWARM_VEHICLE": context["vehicle"],
            "AEROHUB_SWARM_VEHICLE_INDEX": context["vehicle_index"],
            "AEROHUB_SWARM_CENTER": context["center_vehicle"],
            "AEROHUB_SWARM_SCENARIO": context["scenario"],
            "AEROHUB_SWARM_VEHICLE_COUNT": context["vehicle_count"],
            "AEROHUB_SWARM_PLACEMENT": placement,
            "AEROHUB_SWARM_ROLE": role,
        }
    )
    return result


def _scenario(topology: dict[str, Any], name: str) -> dict[str, Any]:
    for scenario in topology["scenarios"]:
        if scenario["name"] == name:
            return scenario
    choices = ", ".join(item["name"] for item in topology["scenarios"])
    raise ConfigError(f"unknown scenario {name}; choose one of: {choices}")


def build_services(
    config: dict[str, Any],
    topology: dict[str, Any],
    scenario: dict[str, Any],
    output: Path,
    stop_event: threading.Event,
) -> list[ServiceThread]:
    services = []
    board = topology["board_vehicle"]
    if board is not None:
        board_context = command_context(topology, scenario, board, output)
        board_role = (
            "center" if scenario["center_vehicle"] == board["name"] else scenario["board_role"]
        )
        services.append(
            ServiceThread(
                identity=f"board-launcher-{board['name']}",
                placement="physical_board",
                command=format_command(
                    config["nodes"]["board"]["launch_command"], board_context
                ),
                environment=_environment(
                    os.environ, board_context, "physical_board", board_role
                ),
                log_path=output / f"board-{board['name']}.log",
                stop_event=stop_event,
            )
        )
    for vehicle in topology["onboard_computer_vehicles"]:
        context = command_context(topology, scenario, vehicle, output)
        role = "center" if scenario["center_vehicle"] == vehicle["name"] else "worker"
        services.append(
            ServiceThread(
                identity=f"onboard-{vehicle['name']}",
                placement="onboard_computer_container",
                command=format_command(
                    config["nodes"]["onboard_computer"]["launch_command"], context
                ),
                environment=_environment(
                    os.environ, context, "onboard_computer_container", role
                ),
                log_path=output / f"onboard-{vehicle['name']}.log",
                stop_event=stop_event,
            )
        )
    if config["ground_station"]["required"]:
        context = command_context(topology, scenario, None, output)
        services.append(
            ServiceThread(
                identity="ground-station",
                placement="pc-ground-station",
                command=format_command(config["ground_station"]["launch_command"], context),
                environment=_environment(os.environ, context, "pc", "ground_station"),
                log_path=output / "ground-station.log",
                stop_event=stop_event,
            )
        )
    return services


def run_scenario(
    config: dict[str, Any],
    scenario_name: str,
    output: Path,
    duration_s: float | None = None,
) -> dict[str, Any]:
    topology = resolve_topology(config)
    scenario = _scenario(topology, scenario_name)
    output.mkdir(parents=True, exist_ok=True)
    report_path = output / "report.json"
    if report_path.exists():
        raise ConfigError(f"output already contains a report: {report_path}")
    duration = float(
        duration_s if duration_s is not None else config["validation"]["smoke_duration_s"]
    )
    startup_timeout = float(config["validation"].get("startup_timeout_s", 15.0))
    stop_event = threading.Event()
    services = build_services(config, topology, scenario, output, stop_event)
    started = time.time()
    try:
        for service in services:
            service.start()
        deadline = time.monotonic() + startup_timeout
        for service in services:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not service.ready.wait(remaining):
                service.error = service.error or "startup timeout"
        if any(service.error for service in services):
            raise RuntimeError("one or more swarm services failed during startup")
        onboard_threads = [
            service.thread_id for service in services
            if service.identity.startswith("onboard-")
        ]
        expected = topology["invariants"]["onboard_computer_vehicle_count"]
        if len(onboard_threads) != expected:
            raise RuntimeError("onboard-computer thread count does not match deployment stage")
        if len(set(onboard_threads)) != len(onboard_threads):
            raise RuntimeError("onboard-computer nodes lack independent supervisor threads")
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            failures = [service for service in services if service.exited_before_stop or service.error]
            if failures:
                raise RuntimeError("one or more swarm services exited before the smoke window")
            time.sleep(0.05)
        passed = True
        failure = None
    except BaseException as exc:
        passed = False
        failure = f"{type(exc).__name__}: {exc}"
    finally:
        stop_event.set()
        for service in services:
            service.join(timeout=8)
        for service in services:
            if service.is_alive():
                passed = False
                failure = failure or f"service thread did not stop: {service.identity}"
    finished = time.time()
    records = [service.record() for service in services]
    report = {
        "schema_version": 1,
        "scenario": scenario,
        "passed": passed,
        "failure": failure,
        "started_unix_s": started,
        "finished_unix_s": finished,
        "duration_s": finished - started,
        "topology": topology,
        "services": records,
        "assertions": {
            "board_vehicle_count": topology["invariants"]["board_vehicle_count"],
            "onboard_computer_vehicle_count": sum(
                record["identity"].startswith("onboard-") for record in records
            ),
            "onboard_computer_threads_are_independent": len(
                {
                    record["supervisor_thread_id"] for record in records
                    if record["identity"].startswith("onboard-")
                }
            ) == topology["invariants"]["onboard_computer_vehicle_count"],
            "ground_station_on_pc": (
                not config["ground_station"]["required"]
                or any(record["placement"] == "pc-ground-station" for record in records)
            ),
        },
    }
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    return report


def run_matrix(
    config: dict[str, Any], output: Path, duration_s: float | None = None
) -> dict[str, Any]:
    topology = resolve_topology(config)
    output.mkdir(parents=True, exist_ok=True)
    reports = []
    for scenario in topology["scenarios"]:
        reports.append(
            run_scenario(config, scenario["name"], output / scenario["name"], duration_s)
        )
    matrix = {
        "schema_version": 1,
        "coordination_mode": topology["coordination_mode"],
        "required_scenarios": [scenario["name"] for scenario in topology["scenarios"]],
        "reports": reports,
        "passed": all(report["passed"] for report in reports),
    }
    (output / "matrix-report.json").write_text(json.dumps(matrix, indent=2) + "\n")
    return matrix


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("run", "run-matrix"))
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--scenario")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float)
    args = parser.parse_args()
    config = load_config(args.config)
    if args.action == "run":
        if not args.scenario:
            raise ConfigError("run requires --scenario")
        result = run_scenario(config, args.scenario, args.output, args.duration)
    else:
        result = run_matrix(config, args.output, args.duration)
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ConfigError, OSError) as exc:
        raise SystemExit(str(exc))
