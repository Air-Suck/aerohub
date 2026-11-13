# SPDX-License-Identifier: GPL-3.0-or-later
"""YOLO26n-seg ROS inference facade; independent of Elastic and task selection."""
import json
import threading
import time
from array import array

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from uav_inference_interfaces.msg import Detection, DetectionFrame
from uav_inference_interfaces.srv import InferDetections

from .yolo_client import HttpDetector
from .yolo_contract import seconds, validate_frame, validate_request


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_inference_adapter')
        endpoint = self.declare_parameter('endpoint', 'http://127.0.0.1:8002').value
        timeout = self.declare_parameter('timeout_s', .4).value
        if not isinstance(timeout, (int, float)) or not 0 < timeout < float('inf'):
            raise ValueError('timeout_s must be positive finite seconds')
        self.detector = HttpDetector(dict(endpoint=endpoint, http_timeout=timeout,
            model_type='yolo26n-seg', class_map={k: k for k in ('person', 'car', 'truck', 'bus')}))
        self.lock = threading.Lock()
        self.last_call = {}
        self.status = self.create_publisher(String, 'inference/yolo/status', 10)
        self.create_service(InferDetections, 'inference/yolo/infer', self.infer,
                            callback_group=ReentrantCallbackGroup())
        self.create_timer(.5, self.health)

    def health(self):
        try:
            metadata = self.detector.client.request('/health/ready')
            if (metadata['worker_generation'] != self.detector.worker_generation or
                    metadata['model_version'] != self.detector.model_version):
                raise ValueError('worker changed: restart adapter and invalidate caller sessions')
            status = dict(metadata, ready=True, last_call=self.last_call)
        except Exception as error:
            status = dict(ready=False, error=str(error), last_call=self.last_call)
        self.status.publish(String(data=json.dumps(status)))

    def infer(self, request, response):
        if not self.lock.acquire(blocking=False):
            response.error = 'BUSY'
            return response
        started = time.perf_counter()
        try:
            image = validate_request(request)
            converted = time.perf_counter()
            source = request.image
            self.last_call = dict(request_id=request.request_id, source_stamp=seconds(source.header.stamp))
            boxes, transform = self.detector.detect(image,
                source.header.stamp.sec * 1_000_000_000 + source.header.stamp.nanosec,
                source.header.frame_id, request.generation, request.request_id)
            frame = DetectionFrame(header=source.header, request_id=request.request_id,
                generation=request.generation, worker_generation=self.detector.worker_generation,
                model_version=self.detector.model_version, width=source.width, height=source.height,
                valid=True, transform_json=json.dumps(transform), timings_json=json.dumps(dict(
                    self.detector.timings, ros_image_conversion_ms=(converted-started)*1000,
                    adapter_total_ms=(time.perf_counter()-started)*1000)))
            for box in boxes:
                detection = Detection(xyxy=box['xyxy'], category=box['category'], confidence=box['confidence'])
                mask = box['mask']
                detection.mask = Image(header=source.header, width=source.width, height=source.height,
                    encoding='mono8', step=source.width, data=array('B', mask.tobytes()))
                frame.detections.append(detection)
            validate_frame(frame, request)
            response.result, response.success = frame, True
        except Exception as error:
            response.error = str(error)
            self.last_call['error'] = str(error)
        finally:
            self.last_call['adapter_total_ms'] = (time.perf_counter()-started)*1000
            self.lock.release()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
