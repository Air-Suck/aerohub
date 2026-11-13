# SPDX-License-Identifier: GPL-3.0-or-later
"""ROS RGB-D Object Detection Tool; no target KF, prediction or motion control."""
from array import array
import json
import threading
import time
import uuid
import cv2
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from std_srvs.srv import SetBool
from uav_inference_interfaces.srv import InferDetections
from uav_object_detection_interfaces.msg import ObjectDetectionFrame
from uav_object_detection_interfaces.srv import DetectObjects
from uav_inference.yolo_contract import array_image, seconds, validate_frame
from tf2_ros import Buffer, TransformListener
from .projection import Projector


class ObjectDetectionTool(Node):
    def __init__(self):
        super().__init__('object_detection_tool')
        self.category=self.declare_parameter('target_class','person').value
        if self.category not in ('person','car','truck','bus'):raise ValueError('unsupported task category')
        self.optical=self.declare_parameter('optical_frame','').value
        self.timeout=self.declare_parameter('timeout_s',.4).value
        if not 0 < self.timeout < float('inf'):raise ValueError('timeout_s must be positive finite')
        self.streaming=self.declare_parameter('stream_enabled',False).value
        image_topic=self.declare_parameter('image_topic','sensor_measurements/FrontCamera/rgb/image').value
        namespace=self.get_namespace().strip('/')
        output_frame=self.declare_parameter('output_frame',namespace+'/odom' if namespace else 'odom').value
        self.tf_buffer=Buffer(node=self)
        self.tf_listener=TransformListener(self.tf_buffer,self)
        self.projector=Projector(self.tf_buffer,output_frame,
            sync_tolerance=self.declare_parameter('sync_tolerance',.05).value,
            transform_wait=self.declare_parameter('transform_wait',.25).value,
            scale_16u=self.declare_parameter('depth_scale_16u',.001).value,
            perspective=self.declare_parameter('depth_is_perspective',False).value,
            registered=self.declare_parameter('registered_depth',False).value)
        for kind,default in [('depth_topic','sensor_measurements/FrontCamera/depth_planar/image'),
                             ('rgb_info_topic','sensor_measurements/FrontCamera/rgb/camera_info'),
                             ('depth_info_topic','sensor_measurements/FrontCamera/depth_planar/camera_info')]:
            topic=self.declare_parameter(kind,default).value
            callback=self.on_depth if kind=='depth_topic' else lambda msg,key=kind:self.on_info('rgb' if key=='rgb_info_topic' else 'depth',msg)
            self.create_subscription(Image if kind=='depth_topic' else CameraInfo,topic,callback,qos_profile_sensor_data)
        self.generation=time.time_ns();self.enabled=True
        self.lock=threading.RLock();self.call_lock=threading.Lock()
        self.pending=None;self.last_rgb=self.last_clock=0.
        self.dropped=self.processed=self.rejected=0
        self.last_success=0.;self.error='waiting for camera';self.model={}
        self.last_call={}
        self.stop_event=threading.Event()
        group=ReentrantCallbackGroup()
        self.client=self.create_client(InferDetections,'inference/yolo/infer',callback_group=group)
        self.frames=self.create_publisher(ObjectDetectionFrame,'perception/object_detection/detections',10)
        self.debug=self.create_publisher(String,'perception/object_detection/debug',10)
        self.health=self.create_publisher(String,'tools/object_detection/status',10)
        self.overlay=self.create_publisher(Image,'perception/object_detection/overlay',qos_profile_sensor_data)
        if self.streaming:
            self.create_subscription(Image,image_topic,self.on_rgb,qos_profile_sensor_data)
        self.create_subscription(String,'inference/yolo/status',lambda m:setattr(self,'model',json.loads(m.data)),10)
        self.create_service(DetectObjects,'tools/object_detection/detect',self.service,callback_group=group)
        self.create_service(SetBool,'tools/object_detection/enable',self.enable,callback_group=group)
        self.create_timer(.05,self.tick)
        self.worker=threading.Thread(target=self.work,daemon=True);self.worker.start()

    def now(self):return self.get_clock().now().nanoseconds/1e9

    def enable(self,request,response):
        with self.lock:
            self.enabled=bool(request.data);self.pending=None
            self.generation=max(time.time_ns(),self.generation+1);self.last_rgb=0.
            self.error='' if self.enabled else 'STOPPED'
            self.frames.publish(ObjectDetectionFrame(tool_generation=self.generation,valid=False,error='tool generation changed'))
        response.success=True
        response.message='Enabled with a new generation' if self.enabled else 'Stopped; pending results discarded, worker computation may finish'
        return response

    def on_depth(self,msg):
        try:self.projector.on_depth(msg)
        except ValueError as error:self.error=str(error);self.rejected+=1

    def on_info(self,kind,msg):
        try:self.projector.on_info(kind,msg)
        except ValueError as error:self.error=str(error);self.rejected+=1

    def active(self,token):
        with self.lock:return self.enabled and token==self.generation

    def on_rgb(self,msg):
        stamp=seconds(msg.header.stamp)
        with self.lock:
            if not self.enabled:return
            if stamp<=0 or stamp<=self.last_rgb:
                self.rejected+=1;return
            self.last_rgb=stamp
            if self.pending is not None:self.dropped+=1
            self.pending=(self.generation,msg)

    def infer(self,request):
        if not self.call_lock.acquire(blocking=False):raise ValueError('BUSY')
        try:
            with self.lock:
                token=self.generation
                if not self.enabled:raise ValueError('STOPPED')
            if not request.image.header.frame_id or (self.optical and request.image.header.frame_id!=self.optical):raise ValueError('unexpected/missing optical frame')
            if not self.client.service_is_ready():raise ValueError('INFERENCE_NOT_READY')
            started=time.perf_counter();done=threading.Event()
            self.last_call=dict(request_id=request.request_id,send_monotonic_s=started,
                request_age_s=self.now()-seconds(request.image.header.stamp))
            inference_request=InferDetections.Request(request_id=request.request_id,generation=request.generation,image=request.image)
            future=self.client.call_async(inference_request);future.add_done_callback(lambda _:done.set())
            if not done.wait(self.timeout+.15):
                future.cancel();raise ValueError('INFERENCE_TIMEOUT; no retry')
            result=future.result()
            self.last_call.update(receive_monotonic_s=time.perf_counter(),
                response_age_s=self.now()-seconds(request.image.header.stamp),
                roundtrip_ms=(time.perf_counter()-started)*1000)
            if result is None or not result.success:raise ValueError(result.error if result else 'INFERENCE_FAILED')
            with self.lock:
                if not self.enabled or token!=self.generation:raise ValueError('CANCELLED_GENERATION')
            frame=result.result
            validate_frame(frame,request)
            frame.detections=[b for b in frame.detections if b.category==self.category]
            timing=json.loads(frame.timings_json)
            timing['tool_ros_roundtrip_ms']=(time.perf_counter()-started)*1000
            frame.timings_json=json.dumps(timing)
            measured=self.projector.project(frame,token,self.active)
            if not self.active(token):raise ValueError('CANCELLED_GENERATION')
            return measured
        except Exception as exc:
            self.last_call['error']=str(exc)
            raise
        finally:self.call_lock.release()

    def service(self,request,response):
        try:
            response.result=self.infer(request);response.success=response.result.valid
            response.error=response.result.error
            self.error=response.error
        except Exception as exc:response.error=str(exc)
        return response

    def work(self):
        while not self.stop_event.wait(.005):
            with self.lock:
                if self.pending is None:continue
                generation,msg=self.pending;self.pending=None
            try:
                frame=self.infer(DetectObjects.Request(request_id=uuid.uuid4().hex,generation=generation,image=msg))
                with self.lock:
                    if generation!=self.generation:continue
                    self.frames.publish(frame)
                boxes=[dict(xyxy=list(m.detection.xyxy),category=m.detection.category,confidence=m.detection.confidence,
                    position_valid=m.position_valid,position=[m.position.x,m.position.y,m.position.z]) for m in frame.detections]
                self.debug.publish(String(data=json.dumps(dict(stamp=seconds(msg.header.stamp),boxes=boxes,
                    generation=generation,model_version=frame.model_version,timings=json.loads(frame.timings_json)))))
                image=array_image(msg)
                for b in boxes:
                    x1,y1,x2,y2=map(int,b['xyxy']);cv2.rectangle(image,(x1,y1),(x2,y2),(0,255,0),2)
                    cv2.putText(image,b['category'],(x1,y1+14),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0),1)
                self.overlay.publish(Image(header=msg.header,width=msg.width,height=msg.height,encoding='bgr8',step=msg.width*3,data=array('B',image.tobytes())))
                self.processed+=1;self.last_success=time.monotonic();self.error=frame.error
            except Exception as exc:
                with self.lock:
                    if generation!=self.generation:continue
                    self.error=str(exc);self.rejected+=1
                    self.frames.publish(ObjectDetectionFrame(image_header=msg.header,generation=generation,tool_generation=generation,valid=False,error=self.error))
                self.debug.publish(String(data=json.dumps(dict(stamp=seconds(msg.header.stamp),boxes=[],error=self.error))))

    def tick(self):
        now=self.now()
        with self.lock:
            if now<self.last_clock:
                self.generation=max(time.time_ns(),self.generation+1);self.pending=None;self.last_rgb=0.
                self.projector.reset();self.tf_buffer.clear()
            self.last_clock=now
            self.health.publish(String(data=json.dumps(dict(healthy=self.enabled and self.worker.is_alive() and
                not self.error,error=self.error,generation=self.generation,
                model=self.model,last_call=self.last_call,target_class=self.category,processed=self.processed,dropped=self.dropped,rejected=self.rejected))))


def main(args=None):
    rclpy.init(args=args)
    tool=ObjectDetectionTool()
    executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(tool)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        tool.stop_event.set();tool.worker.join(timeout=2);executor.shutdown()
        tool.destroy_node();rclpy.try_shutdown()


if __name__ == '__main__':
    main()
