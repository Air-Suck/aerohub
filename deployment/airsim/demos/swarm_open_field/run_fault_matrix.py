#!/usr/bin/env python3
"""Run the two in-flight Swarm process-failure scenarios and preserve evidence."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time


SCENARIOS = (
    ("worker_planner_exit", "board_center", "swarm_worker_planner_exit"),
    ("center_proxy_exit", "board_worker", "swarm_center_proxy_exit"),
)


def validate_evidence(output):
    result_path = output / "result.json"
    workers_path = output / "pc-worker-final.json"
    board_path = output / "board-cleanup.json"
    scene_path = output / "scene/state.json"
    required = (result_path, workers_path, board_path, scene_path)
    if not all(path.is_file() for path in required) or (output / "cleanup-errors.json").exists():
        return False
    result = json.loads(result_path.read_text())
    workers = json.loads(workers_path.read_text())
    board = json.loads(board_path.read_text())
    scene = json.loads(scene_path.read_text())
    return (
        result.get("planner_status") == "FAILED"
        and result.get("as2_hover_confirmed") is True
        and result.get("landed_and_disarmed") is True
        and all(
            not worker.get("error")
            and worker.get("thread_alive") is False
            and worker.get("container_present") is False
            for worker in workers
        )
        and not board.get("remaining")
        and scene.get("phase") == "stopped"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--swarm-config", type=Path)
    parser.add_argument("--compute-image", default="uav/swarm-hybrid-compute:local")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    report_path = output / "fault-matrix.json"
    if report_path.exists():
        raise RuntimeError("choose a fresh fault-matrix output directory")
    runner = Path(__file__).with_name("run_robopi.py")
    reports = []
    for name, coordination, fault in SCENARIOS:
        scenario_output = output / name
        command = [
            sys.executable, str(runner),
            "--output", str(scenario_output),
            "--compute-image", args.compute_image,
            "--coordination-scenario", coordination,
            "--fault-scenario", fault,
        ]
        if args.swarm_config:
            command += ["--swarm-config", str(args.swarm_config.resolve())]
        started = time.time()
        completed = subprocess.run(command, text=True, capture_output=True)
        scenario_output.mkdir(parents=True, exist_ok=True)
        (scenario_output / "matrix-stdout.log").write_text(completed.stdout)
        (scenario_output / "matrix-stderr.log").write_text(completed.stderr)
        result_path = scenario_output / "result.json"
        passed = completed.returncode == 0 and validate_evidence(scenario_output)
        reports.append({
            "name": name,
            "coordination_scenario": coordination,
            "fault_scenario": fault,
            "command": command,
            "returncode": completed.returncode,
            "started_unix_s": started,
            "finished_unix_s": time.time(),
            "result": str(result_path),
            "passed": passed,
        })
    report = {
        "schema_version": 1,
        "reports": reports,
        "passed": all(item["passed"] for item in reports),
    }
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        raise SystemExit(str(exc))
