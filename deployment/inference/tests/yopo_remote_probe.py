"""Standalone workstation caller. Adapter/worker must run in separate processes."""
import copy
import json
import statistics
import struct
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from uav_inference_interfaces.srv import InferYopo


def main():
    rclpy.init()
    node = Node('yopo_remote_probe')
    timings = {}
    node.create_subscription(String, "/drone0/inference/yopo/timing",
                             lambda m: timings.update({json.loads(m.data)["request_id"]: json.loads(m.data)}), 10)
    client = node.create_client(InferYopo, '/drone0/inference/yopo/infer')
    try:
        assert client.wait_for_service(timeout_sec=15), 'adapter unavailable'
        durations = []
        for i in range(10):
            request = InferYopo.Request()
            request.request_id = 'remote-probe-'+str(i)
            request.observation_header.stamp = node.get_clock().now().to_msg()
            request.observation_header.frame_id = 'drone0/odom'
            request.state_header = copy.deepcopy(request.observation_header)
            ns = request.state_header.stamp.nanosec+200000000
            request.state_header.stamp.sec += ns//1000000000
            request.state_header.stamp.nanosec = ns%1000000000
            request.state_header.frame_id = 'drone0/yopo_aligned'
            request.depth.header = copy.deepcopy(request.observation_header)
            request.depth.header.frame_id = 'drone0/yopo_front_optical'
            request.depth.width, request.depth.height = 320, 185
            request.depth.encoding, request.depth.step = '32FC1', 1280
            request.depth.data = struct.pack('<f', 20.) * (320*185)
            request.velocity.x, request.acceleration.y, request.relative_goal.x = 1.2, .2, 15.
            started = time.perf_counter()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            assert future.done(), 'DDS timeout'
            result = future.result()
            elapsed = (time.perf_counter()-started)*1000
            assert result.success, (result.error_code, result.message)
            assert result.request_id == request.request_id and result.header == request.state_header
            assert result.observation_header == request.observation_header
            assert result.device == 'cuda', result.device
            assert len(result.candidates) == 15
            assert result.best_index == min(range(15), key=lambda j: result.candidates[j].score)
            durations.append(elapsed)
        deadline = time.monotonic()+2
        while len(timings) < 10 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        assert len(timings) == 10, "missing component timing messages"
        for details in timings.values():
            assert details['success'] and all(key in details['worker_ms'] for key in ['preprocess','forward_and_transfer','decode'])
            assert all(value >= 0 for value in details['worker_ms'].values())
        print(json.dumps(dict(component_timings=timings, result='passed', samples_ms=durations, p50_ms=statistics.median(durations),
                              max_ms=max(durations), below_100ms=sum(d < 100 for d in durations),
                              model_version=result.model_version, device=result.device,
                              scope='synthetic input; workstation DDS -> onboard Adapter -> GPU worker')))
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
