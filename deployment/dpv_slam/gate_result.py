#!/usr/bin/env python3
"""Evaluate a DPV-SLAM run against the predeclared acceptance gate."""

import argparse
import json
from pathlib import Path

import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("gate", type=Path)
    parser.add_argument("result", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    gate = yaml.safe_load(args.gate.read_text())
    result = json.loads(args.result.read_text())
    gates = gate["gates"]
    checks = {
        "minimum_airborne_seconds": (
            result["airborne_duration_s"], gate["duration"]["minimum_airborne_seconds"], ">="),
        "minimum_horizontal_truth_motion_m": (
            result["truth_max_horizontal_movement_m"],
            gate["duration"]["minimum_horizontal_truth_motion_m"], ">="),
        "image_input_rate_min_hz": (
            result["image_input_rate_hz"], gates["image_input_rate_min_hz"], ">="),
        "candidate_output_rate_min_hz": (
            result["candidate_output_rate_hz"], gates["candidate_output_rate_min_hz"], ">="),
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
        "alignment_scale_min": (
            result["alignment_scale"], gates["alignment_scale_min"], ">="),
        "alignment_scale_max": (
            result["alignment_scale"], gates["alignment_scale_max"], "<="),
        "alignment_rmse_max_m": (
            result["alignment_rmse_m"], gates["alignment_rmse_max_m"], "<="),
        "tracking_loss_events_max": (
            result["tracking_loss_events"], gates["tracking_loss_events_max"], "<="),
        "non_monotonic_output_count_max": (
            result["non_monotonic_output_count"],
            gates["non_monotonic_output_count_max"], "<="),
        "sparse_point_samples_min": (
            result["sparse_point_samples"], gates["sparse_point_samples_min"], ">="),
        "landed_and_disarmed": (result["landed_and_disarmed"], True, "=="),
    }
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
