"""Read-only policy inference service; never publishes motion references."""

import json
import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from uav_inference_interfaces.srv import InferDiffphys

from .diffphys_contract import request_payload, validate_result
from .model_client import ModelClient, WorkerError


class DiffphysNode(Node):
    def __init__(self):
        super().__init__("diffphys_inference")
        endpoint = self.declare_parameter("endpoint", "http://127.0.0.1:8001").value
        timeout = self.declare_parameter("timeout_s", 0.1).value
        self.max_age = self.declare_parameter("max_input_age_s", 0.5).value
        self.max_skew = self.declare_parameter("max_input_skew_s", 0.05).value
        if any(not math.isfinite(value) for value in (timeout, self.max_age, self.max_skew)) or min(timeout, self.max_age) <= 0 or self.max_skew < 0:
            raise ValueError("timeout/age must be positive and skew nonnegative finite seconds")
        self.client = ModelClient(endpoint, timeout)
        timing_path = self.declare_parameter('timing_file', '').value
        self.timing = open(timing_path, 'w', buffering=1) if timing_path else None
        self.lock = threading.Lock()
        self.group = ReentrantCallbackGroup()
        self.service = self.create_service(InferDiffphys, "inference/diffphys/infer", self.infer, callback_group=self.group)
        self.status = self.create_publisher(String, "inference/diffphys/status", 1)
        self.timer = self.create_timer(1., self.health)

    def health(self):
        try:
            metadata = self.client.request("/health/ready")
            metadata["ready"] = True
        except WorkerError as error:
            metadata = dict(ready=False, error_code=error.code, message=str(error))
        self.status.publish(String(data=json.dumps(metadata)))

    def infer(self, request, response):
        def mark(stage, **values):
            if self.timing:
                self.timing.write(json.dumps(dict(request_id=request.request_id,stage=stage,wall=time.monotonic(),**values))+'\n')
        mark('received')
        response.request_id = request.request_id
        response.state_header, response.depth_header = request.state_header, request.depth.header
        if not self.lock.acquire(blocking=False):
            response.error_code, response.message = "BUSY", "another request is active"
            return response
        try:
            payload = request_payload(request, self.get_clock().now().nanoseconds, self.max_age, self.max_skew)
            mark('http_start')
            result = self.client.request("/infer", payload)
            mark('http_end',model_timings_ms=result.get('timings_ms'))
            try:
                validate_result(result, request.request_id)
            except (KeyError, ValueError, TypeError, OverflowError) as error:
                raise WorkerError("INVALID_RESULT", str(error)) from error
            # Repeat freshness checks after computation, including clock rollback.
            try:
                request_payload(request, self.get_clock().now().nanoseconds, self.max_age, self.max_skew)
            except ValueError as error:
                raise WorkerError("STALE_RESULT", str(error)) from error
            response.action = result["action"]
            response.hidden_state = result["hidden_state"]
            response.model_version, response.device = result["model_version"], result["device"]
            response.success = True
        except ValueError as error:
            response.error_code, response.message = "INVALID_INPUT", str(error)
        except WorkerError as error:
            response.error_code, response.message = error.code, str(error)
        finally:
            self.lock.release()
            mark('response_ready',success=response.success,error=response.error_code)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DiffphysNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if node.timing: node.timing.close()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
