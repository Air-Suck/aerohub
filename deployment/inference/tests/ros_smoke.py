"""Actual DDS -> adapter -> HTTP -> model round trip, using synthetic inputs."""

import copy
import json
import struct

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from uav_inference.yopo_node import YopoNode
from uav_inference_interfaces.srv import InferYopo


def main():
    rclpy.init()
    adapter = YopoNode()
    caller = Node("inference_smoke_client")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(adapter)
    executor.add_node(caller)
    try:
        client = caller.create_client(InferYopo, "inference/yopo/infer")
        assert client.wait_for_service(timeout_sec=5), "ROS service unavailable"
        request = InferYopo.Request()
        request.request_id = "ros-smoke"
        request.state_header.stamp = caller.get_clock().now().to_msg()
        request.state_header.frame_id = "aligned_body"
        request.observation_header = copy.deepcopy(request.state_header)
        request.depth.header.stamp = request.state_header.stamp
        request.depth.header.frame_id = "camera_depth_optical"
        request.depth.encoding = "32FC1"
        request.depth.height = request.depth.width = 2
        request.depth.step = 8
        request.depth.data = struct.pack("<4f", 20., 20., 20., 20.)
        request.relative_goal.x = 10.
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=15)
        assert future.done(), "ROS inference timeout"
        result = future.result()
        assert result.success, (result.error_code, result.message)
        assert len(result.candidates) == 15, len(result.candidates)
        assert result.header == request.state_header
        assert result.request_id == request.request_id
        assert result.best_index == min(range(15), key=lambda index: result.candidates[index].score)
        # A stale measurement must fail before reaching the worker.
        request.state_header.stamp.sec = 1
        request.depth.header.stamp.sec = 1
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=5)
        assert future.done()
        stale = future.result()
        assert not stale.success and stale.error_code == "INVALID_INPUT", stale
        print(json.dumps({"ros_roundtrip": "passed", "stale_rejection": "passed",
                          "candidates": len(result.candidates), "device": result.device,
                          "model_version": result.model_version}))
    finally:
        executor.shutdown()
        caller.destroy_node()
        adapter.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
