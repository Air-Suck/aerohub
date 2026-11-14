#!/usr/bin/env python3
"""Evaluate a PIN-SLAM run against the predeclared acceptance gate."""

import argparse
import json
from pathlib import Path

import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("gate", type=Path)
    parser.add_argument("result", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--authority", action="store_true",
        help="also enforce canonical state freshness during airborne authority")
    parser.add_argument(
        "--authority-freshness", type=Path,
        help="dedicated canonical freshness report for the authority flight")
    args = parser.parse_args()

    gate = yaml.safe_load(args.gate.read_text())
    result = json.loads(args.result.read_text())
    if args.authority_freshness:
        result.update(json.loads(args.authority_freshness.read_text()))
    gates = gate["gates"]
    checks = {
        "minimum_airborne_seconds": (
            result["airborne_duration_s"],
            gate["duration"]["minimum_airborne_seconds"],
            ">=",
        ),
        "minimum_horizontal_truth_motion_m": (
            result["truth_max_horizontal_movement_m"],
            gate["duration"]["minimum_horizontal_truth_motion_m"],
            ">=",
        ),
        "sample_coverage_min": (
            result["sample_coverage"], gates["sample_coverage_min"], ">="),
        "maximum_output_gap_s": (
            result["maximum_output_gap_s"], gates["maximum_output_gap_s"], "<="),
        "horizontal_rmse_max_m": (
            result["horizontal_rmse_m"], gates["horizontal_rmse_max_m"], "<="),
        "position_rmse_max_m": (
            result["position_rmse_m"], gates["position_rmse_max_m"], "<="),
        "position_p95_max_m": (
            result["position_p95_m"], gates["position_p95_max_m"], "<="),
        "yaw_p95_max_deg": (
            result["yaw_p95_deg"], gates["yaw_p95_max_deg"], "<="),
        "tracking_loss_events_max": (
            result["tracking_loss_events"], gates["tracking_loss_events_max"], "<="),
        "non_monotonic_output_count_max": (
            result["non_monotonic_output_count"],
            gates["non_monotonic_output_count_max"],
            "<=",
        ),
        "neural_point_samples_min": (
            result["neural_point_samples"], gates["neural_point_samples_min"], ">="),
        "landed_and_disarmed": (
            result["landed_and_disarmed"], True, "=="),
    }
    if args.authority:
        threshold = gate["authority_canonical_maximum_gap_s"]
        checks.update({
            "canonical_platform_status_observed": (
                result.get("platform_status_observed"), True, "=="),
            "canonical_airborne_windows_min": (
                result.get("canonical_airborne_windows"), 1, ">="),
            "canonical_window_closed": (
                result.get("canonical_window_open"), False, "=="),
            "canonical_pose_samples_min": (
                result.get("canonical_pose_samples"), 2, ">="),
            "canonical_pose_non_monotonic_count_max": (
                result.get("canonical_pose_non_monotonic_count"), 0, "<="),
            "canonical_pose_source_maximum_gap_s": (
                result.get("canonical_pose_source_maximum_gap_s"), threshold, "<="),
            "canonical_pose_arrival_maximum_gap_s": (
                result.get("canonical_pose_arrival_maximum_gap_s"), threshold, "<="),
        })
    report = {"checks": {}, "passed": True}
    for name, (measured, threshold, operator) in checks.items():
        valid = measured is not None
        if operator == ">=":
            passed = valid and measured >= threshold
        elif operator == "<=":
            passed = valid and measured <= threshold
        else:
            passed = measured == threshold
        report["checks"][name] = {
            "measured": measured,
            "operator": operator,
            "threshold": threshold,
            "passed": passed,
        }
        report["passed"] = report["passed"] and passed

    text = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.write_text(text)
    print(text, end="")
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
