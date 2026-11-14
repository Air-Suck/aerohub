"""Compare success, duration and sampled positions, and report measured optimizer time."""
import json
import math
from pathlib import Path
import statistics
import sys

old, new = [json.loads(Path(p).read_text()) for p in sys.argv[1:3]]
assert len(old) == len(new) == 840, "incomplete benchmark"
max_position_error = max_duration_error = 0.0
for a, b in zip(old, new):
    for key in ("scene", "formation", "drone", "repeat", "success"):
        assert a[key] == b[key], (key, a[key], b[key])
    assert a["success"], ("optimizer failed", a)
    max_duration_error = max(max_duration_error, abs(a["duration"]-b["duration"]))
    assert len(a["samples"]) == len(b["samples"]) == 51
    for p, q in zip(a["samples"], b["samples"]):
        assert all(math.isfinite(x) for x in p+q), "nonfinite sample"
        max_position_error = max(max_position_error, math.dist(p, q))


def timing(values):
    values = sorted(values)
    return {"median_ms": statistics.median(values), "p95_ms": values[math.ceil(len(values)*.95)-1]}


summary = {
    "cases": 56, "repetitions_per_case": 15, "total_calls_per_version": 840,
    "max_position_error_m": max_position_error,
    "max_duration_error_s": max_duration_error,
    "success_ros1": sum(v["success"] for v in old),
    "success_ros2": sum(v["success"] for v in new),
    "timing": {},
}
for mode in (0,1):
    summary["timing"][str(mode)] = {
        "ros1": timing([v["ms"] for v in old if v["formation"] == mode]),
        "ros2": timing([v["ms"] for v in new if v["formation"] == mode]),
    }
print(json.dumps(summary, indent=2))
if len(sys.argv)>3:
    Path(sys.argv[3]).write_text(json.dumps(summary, indent=2)+"\n")
assert max_position_error < 1e-6, "position regression"
assert max_duration_error < 1e-6, "duration regression"
