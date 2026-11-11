#!/usr/bin/env python3
"""Run both required centralized placements and preserve each result."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time


SCENARIOS = ("board_center", "board_worker")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--swarm-config", type=Path)
    parser.add_argument("--compute-image", default="uav/swarm-hybrid-compute:local")
    parser.add_argument("--preflight-only", action="store_true")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    report_path = output / "role-matrix.json"
    if report_path.exists():
        raise RuntimeError("choose a fresh matrix output directory")
    runner = Path(__file__).with_name("run_robopi.py")
    reports = []
    for scenario in SCENARIOS:
        scenario_output = output / scenario
        command = [
            sys.executable, str(runner),
            "--output", str(scenario_output),
            "--compute-image", args.compute_image,
            "--coordination-scenario", scenario,
        ]
        if args.swarm_config:
            command += ["--swarm-config", str(args.swarm_config.resolve())]
        if args.preflight_only:
            command.append("--preflight-only")
        started = time.time()
        completed = subprocess.run(command, text=True, capture_output=True)
        scenario_output.mkdir(parents=True, exist_ok=True)
        (scenario_output / "matrix-stdout.log").write_text(completed.stdout)
        (scenario_output / "matrix-stderr.log").write_text(completed.stderr)
        reports.append(
            {
                "scenario": scenario,
                "command": command,
                "returncode": completed.returncode,
                "started_unix_s": started,
                "finished_unix_s": time.time(),
                "topology": str(scenario_output / "topology.json"),
                "result": str(
                    scenario_output
                    / ("ground-probe.json" if args.preflight_only else "result.json")
                ),
            }
        )
    report = {
        "schema_version": 1,
        "required_scenarios": list(SCENARIOS),
        "preflight_only": args.preflight_only,
        "reports": reports,
        "passed": all(item["returncode"] == 0 for item in reports),
    }
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        raise SystemExit(str(exc))
