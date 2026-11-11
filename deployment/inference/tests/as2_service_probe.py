"""Read-only native AS2 query from the workstation container."""

import json
import statistics
import time

import rclpy
from as2_msgs.srv import ListControlModes
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node("as2_service_baseline_client")
    try:
        endpoint = "/inference_baseline/platform/list_control_modes"
        client = node.create_client(ListControlModes, endpoint)
        if not client.wait_for_service(timeout_sec=20):
            raise RuntimeError("native AS2 service not discovered: " + endpoint)
        times = []
        response = None
        for _ in range(30):
            started = time.perf_counter()
            future = client.call_async(ListControlModes.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            if not future.done():
                raise RuntimeError("AS2 query response timeout")
            response = future.result()
            if response.source != "Platform" or not response.control_modes:
                raise RuntimeError("invalid native AS2 platform response")
            times.append((time.perf_counter() - started) * 1000)
        ordered = sorted(times)
        print(json.dumps({
            "service": endpoint, "service_type": "as2_msgs/srv/ListControlModes",
            "successes": len(times), "source": response.source,
            "control_modes": list(response.control_modes),
            "latency_ms": {"p50": statistics.median(times), "p95": ordered[28], "max": max(times)},
            "samples_ms": times,
        }))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
