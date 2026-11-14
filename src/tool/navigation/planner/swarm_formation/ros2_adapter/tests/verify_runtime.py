"""Feed real native planners and verify seven PolyTraj streams plus JSON sampling."""
import json
import copy
import math
import socket
import struct
import time
from pathlib import Path
import sys

import rclpy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
from traj_utils.msg import PolyTraj

rclpy.init()
node = rclpy.create_node("verify_swarm_humble")
cloud_pub = node.create_publisher(PointCloud2, "/map_generator/global_cloud", 10)
latest = {}
counts = [0]*7
finished = set()


def receive(message):
    if not 0 <= message.drone_id < 7:
        return
    counts[message.drone_id] += 1
    latest[message.drone_id] = message


subscriptions = [node.create_subscription(PolyTraj, f"/drone_{i}_planning/trajectory", receive, 10) for i in range(7)]
subscriptions += [node.create_subscription(Bool, f"/drone_{i}_planning/finish",
                  lambda msg, index=i: finished.add(index) if msg.data else None, 10) for i in range(7)]
positions = [[-12,-4.5,1.5], [-12,-1.5,1.5], [-12,1.5,1.5], [-12,4.5,1.5],
             [-15,-3,1.5], [-15,0,1.5], [-15,3,1.5]]
payload = {"task_id": "humble-migration-validation", "vehicles": [
    {"id": f"drone{i}", "position_enu": p} for i,p in enumerate(positions)],
    "goal_center_enu": [-4,0,1.5], "lead": .2, "sample_dt": .1, "horizon": 1.0}
cloud = PointCloud2()
cloud.header.frame_id = "world"
cloud.height = cloud.width = 1
cloud.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                for name,offset in (("x",0),("y",4),("z",8))]
cloud.point_step = cloud.row_step = 12
cloud.is_dense = True
# One real obstacle, away from this planning corridor; not a mocked planner output.
cloud.data = struct.pack("fff", 15.0, 10.0, 1.0)
deadline = time.monotonic()+90
reply = None
while time.monotonic() < deadline:
    cloud.header.stamp = node.get_clock().now().to_msg()
    cloud_pub.publish(cloud)
    rclpy.spin_once(node, timeout_sec=.02)
    try:
        with socket.create_connection(("127.0.0.1",28501),timeout=2) as sock:
            sock.sendall((json.dumps(payload)+"\n").encode())
            reply = json.loads(sock.makefile("rb").readline())
    except (OSError, ValueError):
        time.sleep(.05)
        continue
    if reply.get("ok") and len(latest)==7 and min(counts)>=2 and len(finished)==7:
        break
    if reply.get("ok"):
        # Replay the requested near-future positions as synthetic odometry.
        # This exercises replanning and completion, without claiming a flight simulation.
        for vehicle, points in zip(payload["vehicles"], reply["trajectories_enu"]):
            vehicle["position_enu"] = points[0]
    time.sleep(.05)
assert reply and reply.get("ok"), ("no complete fleet response", reply, counts)
assert len(latest)==7 and min(counts)>=2, ("not all planners produced rolling trajectories", counts)
assert len(finished)==7, ("completion callback not reached", sorted(finished), counts)
for change in ({"vehicles": []}, {"sample_dt": 0.0}, {"goal_center_enu": [float("nan"),0,1]}):
    invalid = copy.deepcopy(payload)
    invalid.update(change)
    with socket.create_connection(("127.0.0.1",28501),timeout=2) as sock:
        sock.sendall((json.dumps(invalid)+"\n").encode())
        failure = json.loads(sock.makefile("rb").readline())
    assert not failure["ok"] and failure["reason"]=="ERROR", failure
with socket.create_connection(("127.0.0.1",28501),timeout=2) as sock:
    sock.sendall((json.dumps(payload)+"\n").encode())
    assert json.loads(sock.makefile("rb").readline())["ok"], "proxy did not recover"
for i,message in latest.items():
    assert message.order==5
    assert message.duration and all(math.isfinite(t) and t>0 for t in message.duration)
    for coefficients in (message.coef_x,message.coef_y,message.coef_z):
        assert len(coefficients)==6*len(message.duration)
        assert all(math.isfinite(c) for c in coefficients)
trajectories=reply["trajectories_enu"]
assert len(trajectories)==7
for points in trajectories:
    assert len(points)==11
    assert all(len(p)==3 and all(math.isfinite(v) for v in p) for p in points)
summary={"native_trajectory_counts": counts, "json_trajectory_count": len(trajectories),
         "samples_per_vehicle": len(trajectories[0]), "completion_callbacks": sorted(finished),
         "invalid_requests_rejected": 3, "recovery": True}
if len(sys.argv)>1: Path(sys.argv[1]).write_text(json.dumps(summary,indent=2)+"\n")
print("PASS seven real native planners and JSON sampling:",summary)
node.destroy_node()
rclpy.shutdown()
