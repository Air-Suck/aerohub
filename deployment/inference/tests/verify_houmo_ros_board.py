"""Run on RoboPi with its ROS environment and the inference overlay loaded."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import urllib.request

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from uav_inference_interfaces.msg import YopoCandidate
from uav_inference_interfaces.srv import InferYopo


def stop(process):
    if process is not None and process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime', type=Path, required=True)
    parser.add_argument('--log-dir', type=Path, required=True)
    parser.add_argument('--port', type=int, default=18082)
    args = parser.parse_args()
    args.log_dir.mkdir(parents=True, exist_ok=True)
    endpoint = 'http://127.0.0.1:{}'.format(args.port)
    worker = adapter = node = None
    report = {'checks': []}
    # Explicitly exercise the generated extension, not just Python message imports.
    InferYopo.Request.__class__.__import_type_support__()
    YopoCandidate.__class__.__import_type_support__()
    worker_log = (args.log_dir / 'worker.log').open('w')
    adapter_log = (args.log_dir / 'adapter.log').open('w')
    try:
        environment = dict(os.environ, INFERENCE_PORT=str(args.port))
        worker = subprocess.Popen(['/system/bin/sh', str(args.runtime / 'start_board.sh')],
                                  env=environment, stdout=worker_log, stderr=subprocess.STDOUT)
        for attempt in range(100):
            if worker.poll() is not None:
                raise RuntimeError('worker exited: inspect worker.log')
            try:
                with urllib.request.urlopen(endpoint + '/health/ready', timeout=1) as response:
                    report['health'] = json.load(response)
                break
            except OSError:
                time.sleep(0.1)
        else:
            raise RuntimeError('worker readiness timed out')
        adapter = subprocess.Popen([
            '/sys_prod/robot/out/bin/python3', '-m', 'uav_inference.yopo_node',
            '--ros-args', '-p', 'endpoint:=' + endpoint,
        ], stdout=adapter_log, stderr=subprocess.STDOUT)
        rclpy.init()
        node = rclpy.create_node('houmo_ros_verifier')
        diagnostics = []
        subscription = node.create_subscription(
            DiagnosticArray, '/inference/status', lambda message: diagnostics.extend(message.status), 10)
        client = node.create_client(InferYopo, '/inference/yopo/infer')
        if not client.wait_for_service(timeout_sec=15):
            raise RuntimeError('ROS service not discovered: inspect adapter.log')

        def request(identifier, stale=False):
            message = InferYopo.Request()
            message.request_id = identifier
            now = node.get_clock().now().nanoseconds - (10_000_000_000 if stale else 0)
            for header in (message.depth.header, message.observation_header, message.state_header):
                header.stamp.sec, header.stamp.nanosec = divmod(now, 1_000_000_000)
                header.frame_id = 'aligned_body'
            message.depth.height, message.depth.width = 96, 160
            message.depth.encoding, message.depth.step = '32FC1', 160 * 4
            message.depth.data = struct.pack('<' + str(96 * 160) + 'f', *([20.0] * (96 * 160)))
            message.velocity.x, message.relative_goal.x = 1.0, 10.0
            future = client.call_async(message)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            if not future.done():
                raise RuntimeError('ROS call timed out')
            response = future.result()
            assert response.request_id == identifier
            assert response.header == message.state_header
            assert response.observation_header == message.observation_header
            return response

        payload = {'request_id': 'direct-reference', 'depth': [[20.0] * 160 for _ in range(96)],
                   'state': [1., 0., 0., 0., 0., 0., 10., 0., 0.]}
        with urllib.request.urlopen(urllib.request.Request(
                endpoint + '/infer', data=json.dumps(payload).encode(),
                headers={'Content-Type': 'application/json'}), timeout=5) as response:
            direct = json.load(response)
        for index in range(3):
            started = time.perf_counter()
            result = request('ros-hmm-{}'.format(index))
            assert result.success, (result.error_code, result.message)
            assert result.device == 'houmo:xh2:0'
            assert len(result.candidates) == 15
            assert result.best_index == direct['best_index']
            assert result.model_version == direct['model_version']
            assert math.isclose(result.duration_s, direct['duration_s'], rel_tol=1e-9)
            for candidate, expected, score in zip(result.candidates, direct['endstates'], direct['scores']):
                actual = [getattr(vector, axis) for vector in
                          (candidate.position, candidate.velocity, candidate.acceleration)
                          for axis in ('x', 'y', 'z')]
                assert all(math.isfinite(value) for value in actual)
                assert all(math.isclose(value, target, rel_tol=1e-5, abs_tol=1e-6)
                           for value, target in zip(actual, expected))
                assert math.isclose(candidate.score, score, rel_tol=1e-5, abs_tol=1e-6)
            report['checks'].append({'case': 'normal', 'best_index': result.best_index,
                                     'candidates': len(result.candidates), 'device': result.device,
                                     'elapsed_ms': (time.perf_counter() - started) * 1000})
        def diagnostic_ready():
            # diagnostic_msgs uses a ROS byte field; some generators return bytes.
            return any(status.level in (0, b'\x00') and status.message == 'ready'
                       for status in diagnostics)

        deadline = time.monotonic() + 3
        while not diagnostic_ready() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        report['diagnostics'] = [dict(level=repr(status.level), message=status.message)
                                 for status in diagnostics]
        assert diagnostic_ready(), report['diagnostics']
        report['checks'].append({'case': 'diagnostic_ready', 'passed': True})
        result = request('stale-input', stale=True)
        assert not result.success and result.error_code == 'INVALID_INPUT'
        report['checks'].append({'case': 'stale_input', 'error_code': result.error_code})
        stop(worker)
        result = request('worker-offline')
        assert not result.success and result.error_code == 'WORKER_UNAVAILABLE'
        report['checks'].append({'case': 'worker_offline', 'error_code': result.error_code})
        report['passed'] = True
    finally:
        stop(adapter)
        stop(worker)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        worker_log.close()
        adapter_log.close()
        (args.log_dir / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
