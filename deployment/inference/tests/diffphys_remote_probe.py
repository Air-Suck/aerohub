"""Workstation probe for the actual GPU service, without motion or simulator calls."""

import copy
import json
import math
import statistics
import struct
import time

import rclpy
from rclpy.node import Node
from uav_inference_interfaces.srv import InferDiffphys


def main():
    rclpy.init()
    node = Node("diffphys_remote_probe")
    client = node.create_client(InferDiffphys, "/diffphys_probe/inference/diffphys/infer")
    samples = []
    try:
        assert client.wait_for_service(timeout_sec=15), "Adapter unavailable"

        def request(index, hidden=(), encoding="32FC1", bigendian=False):
            result = InferDiffphys.Request()
            result.request_id = "probe-" + str(index)
            result.state_header.stamp = node.get_clock().now().to_msg()
            result.state_header.frame_id = "diffphys_heading_snapshot"
            result.depth.header = copy.deepcopy(result.state_header)
            result.depth.header.frame_id = "front_depth_optical"
            result.depth.height, result.depth.width = 36, 48
            result.depth.encoding = encoding
            result.depth.is_bigendian = bigendian
            fmt, size, value = ("f", 4, 6.) if encoding == "32FC1" else ("H", 2, 6000)
            result.depth.step = 48 * size + 8
            row = struct.pack((">" if bigendian else "<") + fmt, value) * 48 + bytes(8)
            result.depth.data = row * 36
            result.state = [0.4, -0.2, 0.15, 1.1, -0.7, 0.2, 0., 0., 1., 0.15]
            result.hidden_state = list(hidden)
            return result

        def call(req):
            started = time.perf_counter()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
            assert future.done(), "DDS timeout"
            result = future.result()
            assert result.request_id == req.request_id
            assert result.state_header == req.state_header and result.depth_header == req.depth.header
            return result, (time.perf_counter() - started) * 1000

        hidden, first = [], None
        for index in range(20):
            result, elapsed = call(request(index, hidden))
            assert result.success, (result.error_code, result.message)
            assert result.device == "cuda"
            assert len(result.action) == 6 and len(result.hidden_state) == 192
            assert all(math.isfinite(value) for value in list(result.action) + list(result.hidden_state))
            assert elapsed < 100., "ROS roundtrip exceeded predeclared 100ms budget"
            samples.append(elapsed)
            hidden = list(result.hidden_state)
            if index == 0:
                first = result
        # A new empty hidden state reproduces the initial result, across unit/byte-order variants.
        for encoding, bigendian in [("32FC1", True), ("16UC1", False), ("16UC1", True)]:
            reset, _ = call(request("reset", encoding=encoding, bigendian=bigendian))
            assert reset.success, (reset.error_code, reset.message)
            assert list(reset.action) == list(first.action)
            assert list(reset.hidden_state) == list(first.hidden_state)
        invalid = []
        stale = request("stale"); stale.state_header.stamp.sec -= 2; stale.depth.header.stamp.sec -= 2; invalid.append(stale)
        future = request("future"); future.state_header.stamp.sec += 2; future.depth.header.stamp.sec += 2; invalid.append(future)
        skew = request("skew"); skew.depth.header.stamp.sec -= 1; invalid.append(skew)
        bad_hidden = request("bad-hidden", [0.] * 191); invalid.append(bad_hidden)
        nonfinite = request("nonfinite"); nonfinite.state[0] = float("nan"); invalid.append(nonfinite)
        zero = request("zero"); zero.depth.data = bytes(zero.depth.step * 36); invalid.append(zero)
        size = request("size"); size.depth.width = 47; invalid.append(size)
        for item in invalid:
            rejected, _ = call(item)
            assert not rejected.success and rejected.error_code == "INVALID_INPUT", (rejected.error_code, rejected.message)
        print(json.dumps(dict(result="passed", samples_ms=samples, p50_ms=statistics.median(samples),
                              max_ms=max(samples), all_below_100ms=True, device=first.device,
                              model_version=first.model_version, recurrent_steps=20,
                              reset_unit_endian_padding_cases=3, invalid_inputs_rejected=len(invalid),
                              scope="synthetic DDS input; separate workstation/Adapter containers and host GPU worker"), indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
