#!/usr/bin/env python3
"""Expose the CAT-ORA ROS 1 service through a local newline-delimited JSON socket."""

import json
import socketserver

import rospy
from geometry_msgs.msg import Point
from mrs_formation_reshaping.srv import (
    SrvGetReshapingTrajectories,
    SrvGetReshapingTrajectoriesRequest,
)


SERVICE = "/mrs_formation_reshaping/get_reshaping_trajectories"


class NoSolution(RuntimeError):
    pass


def _points(values):
    if not isinstance(values, list) or not values:
        raise ValueError("positions must be a non-empty array")
    result = []
    for value in values:
        if not isinstance(value, list) or len(value) != 3:
            raise ValueError("each position must contain x, y, z")
        result.append(Point(x=float(value[0]), y=float(value[1]), z=float(value[2])))
    return result


def generate(payload):
    starts = _points(payload["starts_enu"])
    goals = _points(payload["goals_enu"])
    if len(starts) != len(goals):
        raise ValueError("start and goal counts differ")
    request = SrvGetReshapingTrajectoriesRequest()
    request.initial_configurations = starts
    request.goal_configurations = goals
    request.trajectory_dt = float(payload["trajectory_dt"])
    request.max_velocity = float(payload["max_velocity"])
    request.max_acceleration = float(payload["max_acceleration"])
    request.publish_visualization = False
    response = rospy.ServiceProxy(SERVICE, SrvGetReshapingTrajectories)(request)
    if not response.success:
        raise NoSolution(response.message)
    if len(response.trajectories) != len(starts):
        raise RuntimeError("CAT-ORA returned an unexpected trajectory count")
    return {
        "trajectories_enu": [
            [[point.x, point.y, point.z] for point in trajectory.waypoints]
            for trajectory in response.trajectories
        ]
    }


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        try:
            payload = json.loads(self.rfile.readline().decode("utf-8"))
            result = {"ok": True, **generate(payload)}
        except NoSolution as exc:
            result = {"ok": False, "reason": "NO_SOLUTION", "error": str(exc)}
        except Exception as exc:
            result = {"ok": False, "error": str(exc)}
        self.wfile.write((json.dumps(result, separators=(",", ":")) + "\n").encode("utf-8"))


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    rospy.init_node("cat_ora_ros1_proxy")
    rospy.wait_for_service(SERVICE)
    host = rospy.get_param("~host", "127.0.0.1")
    port = int(rospy.get_param("~port", 28500))
    with Server((host, port), Handler) as server:
        rospy.loginfo("CAT-ORA proxy listening on %s:%d", host, port)
        server.serve_forever()


if __name__ == "__main__":
    main()
