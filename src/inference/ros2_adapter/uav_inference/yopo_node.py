import math
import time
import json
import threading

import rclpy
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from uav_inference_interfaces.msg import YopoCandidate
from uav_inference_interfaces.srv import InferYopo

from .model_client import ModelClient, WorkerError
from .yopo_contract import request_payload, validate_result


class YopoNode(Node):
    def __init__(self):
        super().__init__("yopo_inference")
        endpoint = self.declare_parameter("endpoint", "http://127.0.0.1:8000").value
        timeout = self.declare_parameter("timeout_s", 5.0).value
        self.max_age = self.declare_parameter("max_input_age_s", 0.5).value
        self.max_future = self.declare_parameter("max_reference_future_s", 0.3).value
        self.max_skew = self.declare_parameter("max_input_skew_s", 0.05).value
        if any(not math.isfinite(value) for value in (timeout, self.max_age, self.max_skew, self.max_future)) or timeout <= 0 or min(self.max_age, self.max_skew, self.max_future) < 0:
            raise ValueError("timeout must be positive; age/skew must be nonnegative finite seconds")
        self.client = ModelClient(endpoint, timeout)
        self.lock = threading.Lock()
        self.service_group = ReentrantCallbackGroup()
        self.health_group = MutuallyExclusiveCallbackGroup()
        self.service = self.create_service(InferYopo, "inference/yopo/infer", self.infer, callback_group=self.service_group)
        self.timings = self.create_publisher(String, "inference/yopo/timing", 10)
        self.diagnostics = self.create_publisher(DiagnosticArray, "inference/status", 1)
        self.timer = self.create_timer(1.0, self.health, callback_group=self.health_group)

    def health(self):
        status = DiagnosticStatus(name=self.get_fully_qualified_name(), hardware_id="yopo")
        try:
            metadata = self.client.request("/health/ready")
            status.level, status.message = DiagnosticStatus.OK, "ready"
            status.values = [KeyValue(key=str(key), value=str(value)) for key, value in metadata.items()]
        except WorkerError as error:
            status.level, status.message = DiagnosticStatus.ERROR, str(error)
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostics.publish(message)

    def infer(self, request, response):
        response.request_id = request.request_id
        response.header = request.state_header
        response.observation_header = request.observation_header
        if not self.lock.acquire(blocking=False):
            response.error_code, response.message = "BUSY", "another inference request is active"
            return response
        started = time.perf_counter()
        details = {}
        try:
            payload = request_payload(request, self.get_clock().now().nanoseconds, self.max_age, self.max_skew, self.max_future)
            converted = time.perf_counter()
            result = self.client.request("/infer", payload)
            received = time.perf_counter()
            details = dict(conversion_ms=(converted-started)*1000, http_ms=(received-converted)*1000,
                           worker_ms=result.get("timings_ms", {}))
            try:
                validate_result(result, request.request_id)
                for state, score in zip(result["endstates"], result["scores"]):
                    candidate = YopoCandidate()
                    candidate.position.x, candidate.position.y, candidate.position.z = state[:3]
                    candidate.velocity.x, candidate.velocity.y, candidate.velocity.z = state[3:6]
                    candidate.acceleration.x, candidate.acceleration.y, candidate.acceleration.z = state[6:9]
                    candidate.score = float(score)
                    response.candidates.append(candidate)
                response.best_index = result["best_index"]
                response.duration_s = float(result["duration_s"])
                response.model_version, response.device = result["model_version"], result["device"]
            except (KeyError, ValueError, TypeError, AssertionError, OverflowError) as error:
                raise WorkerError("INVALID_RESULT", str(error)) from error
            response.success = True
        except ValueError as error:
            response.error_code, response.message = "INVALID_INPUT", str(error)
        except WorkerError as error:
            response.error_code, response.message = error.code, str(error)
        finally:
            details.update(request_id=request.request_id, adapter_ms=(time.perf_counter()-started)*1000,
                           success=response.success, error_code=response.error_code)
            self.timings.publish(String(data=json.dumps(details)))
            self.lock.release()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = YopoNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
