"""Audit recorded command timing; never starts ROS or supplies flight commands."""
import argparse
import bisect
import collections
import json
import math
from pathlib import Path


def distribution(values):
    ordered = sorted(values)
    if not ordered:
        return None
    def percentile(fraction):
        return ordered[min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)]
    return {"count": len(ordered), "p50": percentile(.5),
            "p95": percentile(.95), "p99": percentile(.99), "max": ordered[-1]}


def stream(records):
    result = {"count": len(records)}
    for clock in ("wall", "sim"):
        intervals = [b[clock] - a[clock] for a, b in zip(records, records[1:])]
        result[clock + "_interval_s"] = distribution(intervals)
        result[clock + "_nonpositive_intervals"] = sum(x <= 0 for x in intervals)
        elapsed = records[-1][clock] - records[0][clock] if len(records) > 1 else 0
        result[clock + "_hz"] = (len(records) - 1) / elapsed if elapsed > 0 else None
    return result


def audit(directory):
    data = collections.defaultdict(list)
    with (directory / "samples.jsonl").open() as source:
        for line in source:
            record = json.loads(line)
            data[record["topic"]].append(record)
    px4 = data["fmu/in/vehicle_rates_setpoint"]
    if len(px4) < 2:
        raise ValueError("at least two PX4 input samples required")
    begin, end = px4[0]["wall"], px4[-1]["wall"]
    names = ("external_acro/control", "actuator_command/twist",
             "actuator_command/thrust", "fmu/in/vehicle_rates_setpoint")
    selected = {name: [r for r in data[name] if begin <= r["wall"] <= end]
                for name in names}
    valid = [r for r in selected[names[0]] if r["solver_ok"]]
    # Count observed solution identities, never sender sequence numbers. This
    # is a lower bound: solves overwritten before transmission are not recorded.
    seen = set()
    commands_by_stamp = collections.defaultdict(set)
    distinct = []
    for record in valid:
        stamp = record["last_successful_solve_ns"]
        if stamp <= 0:
            raise ValueError("valid control frame has no successful solve stamp")
        commands_by_stamp[stamp].add(tuple(record["rate"]) + (record["thrust"],))
        if stamp not in seen:
            seen.add(stamp)
            distinct.append(record)
    ages = [r["sim"] - r["last_successful_solve_ns"] / 1e9 for r in valid]
    return {
        "scope": "recorded PX4 input window; receive intervals, not firmware consumption",
        "window_wall_s": [begin, end],
        "streams": {name: stream(records) for name, records in selected.items()},
        "valid_frames": len(valid), "observed_distinct_solutions": stream(distinct),
        "conflicting_solution_stamps": [stamp for stamp, commands in commands_by_stamp.items() if len(commands) > 1],
        "repeated_solution_fraction": 1 - len(distinct) / len(valid) if valid else None,
        "solution_age_at_observer_sim_s": distribution(ages),
        "solution_age_over_100ms_fraction": sum(x > .1 for x in ages) / len(ages) if ages else None,
        "limitations": [
            "A distinct solve stamp identifies a new solution observed in transmitted frames; total solve rate needs health records.",
            "Simulation clock quantization affects ages; receive wall time includes transport and scheduling.",
            "PX4 input publication does not establish firmware consumption rate or unique source pairing.",
            "State/cloud sampling, Behavior callback timing and firmware latency are not measured here.",
        ],
    }


def audit_bag(directory):
    """Requires the recorded ROS message packages; reads SQLite without playback."""
    import sqlite3
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    selected = collections.defaultdict(list)
    costs = []
    stamped_solves = []
    conservative_solves = []
    elapsed_to_health = []
    for path in sorted((directory / "rosbag").glob("*.db3")):
        with sqlite3.connect("file:" + str(path.resolve()) + "?mode=ro", uri=True) as db:
            topics = {i: (name, get_message(kind)) for i, name, kind in
                      db.execute("select id,name,type from topics")
                      if name == "/clock" or name.endswith(("/navigation/odom", "/ipc/processed_points", "/ipc/health"))}
            for topic, stamp, payload in db.execute("select topic_id,timestamp,data from messages order by timestamp"):
                if topic not in topics:
                    continue
                name, cls = topics[topic]
                message = deserialize_message(payload, cls)
                if name.endswith("/health"):
                    if not message.data.startswith("solved "):
                        continue
                    values = dict(item.split("=") for item in message.data.removeprefix("solved ").split(","))
                    costs.append(float(values["wall_ms"]))
                    if 'conservative_corridor' in values:
                        conservative_solves.append(bool(int(values['conservative_corridor'])))
                    if 'solve_stamp_ns' in values:
                        stamped_solves.append({key: int(values[key]) for key in
                            ('solve_stamp_ns', 'state_stamp_ns', 'cloud_stamp_ns')})
                    if 'elapsed_to_health_ms' in values:
                        elapsed_to_health.append(float(values['elapsed_to_health_ms']))
                    sampled = None
                else:
                    value = message.clock if name == "/clock" else message.header.stamp
                    sampled = value.sec + value.nanosec * 1e-9
                selected[name].append((stamp / 1e9, sampled))
    if not selected:
        raise ValueError("no supported SQLite bag records")
    result = {}
    for name, records in selected.items():
        records.sort()
        received = [t for t, _ in records]
        elapsed = received[-1] - received[0]
        sampled = [value for _, value in records if value is not None]
        result[name] = {"count": len(records),
            "record_clock_hz": (len(records) - 1) / elapsed if elapsed > 0 else None,
            "record_interval_s": distribution([b - a for a, b in zip(received, received[1:])]),
            "sample_interval_s": distribution([b - a for a, b in zip(sampled, sampled[1:])])}
    # Health is emitted for every successful computation, unlike a sender that
    # periodically samples the latest cache. Older bags have no solve stamp in
    # health, so bound its simulation time by adjacent recorded clock messages.
    clocks = sorted(selected.get('/clock', []))
    clock_times = [t for t, _ in clocks]
    health = sorted(item for name, records in selected.items() if name.endswith('/ipc/health') for item in records)
    solve_rate = None
    if len(health) > 1 and clocks:
        def bracket(t):
            i = bisect.bisect_right(clock_times, t)
            if i == 0 or i == len(clocks):
                raise ValueError('successful health records are not bracketed by clock records')
            return clocks[i-1][1], clocks[i][1]
        first_low, first_high = bracket(health[0][0])
        last_low, last_high = bracket(health[-1][0])
        duration_min, duration_max = last_low-first_high, last_high-first_low
        solve_rate = {'successful_health_records': len(health),
            'sim_duration_bracket_s': [duration_min, duration_max],
            'sim_hz_bracket': [(len(health)-1)/duration_max, (len(health)-1)/duration_min] if duration_min > 0 else None,
            'scope': 'first through last successful health in the bag; clock receive-order bracket, not a hard synchronization bound'}
    exact = None
    if len(stamped_solves) > 1:
        stamps = sorted(record['solve_stamp_ns'] for record in stamped_solves)
        duration = (stamps[-1]-stamps[0])/1e9
        exact = {'successful_records': len(stamps), 'unique_solve_stamps': len(set(stamps)),
            'sim_hz': (len(stamps)-1)/duration if duration > 0 else None,
            'solve_interval_sim_s': distribution([(b-a)/1e9 for a,b in zip(stamps,stamps[1:])]),
            'state_age_at_solve_sim_s': distribution([(r['solve_stamp_ns']-r['state_stamp_ns'])/1e9 for r in stamped_solves]),
            'cloud_age_at_solve_sim_s': distribution([(r['solve_stamp_ns']-r['cloud_stamp_ns'])/1e9 for r in stamped_solves])}
    return {"scope": "whole bag; recorder clock and sample stamp intervals; not the command window",
            "streams": result, "backend_solve_wall_ms": distribution(costs),
            "successful_computation_rate": solve_rate,
            "stamped_successful_computations": exact,
            "elapsed_to_health_wall_ms": distribution(elapsed_to_health),
            "conservative_corridor_solves": {"observed_records": len(conservative_solves), "count": sum(conservative_solves)},
            "limitations": "Health costs cover SFC/MPC/control conversion reporting boundary, not the entire timer callback. Recorded health messages may be lost; ROS time quantization can hide multiple solves under one solution stamp."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("evidence", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--require-observed-solve-hz", type=float)
    parser.add_argument("--bag", action="store_true", help="Also inspect SQLite bag using installed ROS message packages")
    args = parser.parse_args()
    report = audit(args.evidence)
    if args.bag:
        report["bag"] = audit_bag(args.evidence)
    measured = report["observed_distinct_solutions"]["sim_hz"]
    report["required_observed_solve_sim_hz"] = args.require_observed_solve_hz
    report["observed_solve_rate_passed"] = (None if args.require_observed_solve_hz is None
        else measured is not None and measured >= args.require_observed_solve_hz)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if report["observed_solve_rate_passed"] is False:
        raise SystemExit(1)
    if report["conflicting_solution_stamps"]:
        raise SystemExit("multiple control values share a successful solve timestamp")


if __name__ == "__main__":
    main()
