"""Exercise the native ROS 2 service and JSON compatibility endpoint against ROS 1."""
import copy
import json
import math
import socket
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from mrs_formation_reshaping.srv import SrvGetReshapingTrajectories as Service

rclpy.init()
node = rclpy.create_node("verify_catora_humble")
client = node.create_client(Service, "/mrs_formation_reshaping/get_reshaping_trajectories")
assert client.wait_for_service(timeout_sec=20), "native service unavailable"


def call(case):
    request = Service.Request()
    request.initial_configurations = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in case["starts_enu"]]
    request.goal_configurations = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in case["goals_enu"]]
    for name in ("trajectory_dt", "max_velocity", "max_acceleration", "publish_visualization"):
        if name in case:
            setattr(request, name, case[name])
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20)
    assert future.done(), "service timeout"
    return future.result()


def same(actual, expected):
    assert len(actual) == len(expected)
    for trajectory, reference in zip(actual, expected):
        assert len(trajectory) == len(reference)
        for point, target in zip(trajectory, reference):
            assert math.dist(point, target) < 1e-9, (point, target)


baseline = json.loads(Path(sys.argv[1]).read_text())
cases = json.loads(Path(__file__).with_name("cases.json").read_text())
# The ROS service and TCP proxy start independently. Service discovery alone
# does not mean the proxy has opened its listening socket yet.
deadline = time.monotonic() + 20.0
while True:
    try:
        with socket.create_connection(("127.0.0.1", 28500), timeout=1):
            break
    except OSError:
        if time.monotonic() >= deadline:
            raise
        time.sleep(0.1)
for case in cases:
    response = call(case)
    assert response.success, response.message
    trajectories = [[[p.x, p.y, p.z] for p in t.waypoints] for t in response.trajectories]
    key = case["name"] + ("_corrected" if case["name"] == "unequal_constraints" else "")
    same(trajectories, baseline[key])
    for points, start in zip(trajectories, case["starts_enu"]):
        assert math.dist(points[0], start) < 1e-9
        assert min(math.dist(points[-1], goal) for goal in case["goals_enu"]) < 1e-6
        for a, b in zip(points, points[1:]):
            assert math.dist(a, b) / case["trajectory_dt"] <= case["max_velocity"] + 1e-5
    if case["name"] == "unequal_constraints":
        points = trajectories[0]
        dt = case["trajectory_dt"]
        for a, b, c in zip(points, points[1:], points[2:]):
            acceleration = math.sqrt(sum((c[j] - 2*b[j] + a[j])**2 for j in range(3))) / dt**2
            assert acceleration <= case["max_acceleration"] + 1e-5
        assert len(points) != len(baseline[case["name"]][0]), "legacy argument reversal not exposed"
    with socket.create_connection(("127.0.0.1", 28500), timeout=20) as sock:
        sock.sendall((json.dumps(case) + "\n").encode())
        reply = json.loads(sock.makefile("rb").readline())
    assert reply["ok"], reply
    same(reply["trajectories_enu"], trajectories)
    print("PASS native service + JSON endpoint:", case["name"])

for change in (
    {"starts_enu": [], "goals_enu": []},
    {"goals_enu": []},
    {"trajectory_dt": 0.0},
    {"max_velocity": float("nan")},
    {"max_acceleration": -1.0},
    {"starts_enu": [[float("inf"), 0, 1]]},
    {"publish_visualization": True},
):
    invalid = copy.deepcopy(cases[0])
    invalid.update(change)
    response = call(invalid)
    assert not response.success and not response.trajectories, change
assert call(cases[0]).success, "service did not recover after rejected requests"
print("PASS seven invalid requests and subsequent recovery")
node.destroy_node()
rclpy.shutdown()
