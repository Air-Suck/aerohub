#!/usr/bin/env python3
"""One-shot CAT-ORA proxy stub for the ROS 2 adapter smoke test."""

import json
import socketserver
import sys


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        request = json.loads(self.rfile.readline())
        starts = request["starts_enu"]
        goals = request["goals_enu"]
        response = {
            "ok": True,
            "trajectories_enu": [[start, goal] for start, goal in zip(starts, goals)],
        }
        self.wfile.write((json.dumps(response) + "\n").encode())


class Server(socketserver.TCPServer):
    allow_reuse_address = True


with Server(("127.0.0.1", int(sys.argv[1])), Handler) as server:
    server.handle_request()
