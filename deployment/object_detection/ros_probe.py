"""Independent ROS client + sensor fixtures; real CUDA inference, synthetic geometry.

Never starts a simulator or publishes flight commands. Recorded RGB is supplied
by the caller; depth=2m and the camera TF are deliberate software fixtures.
"""
import argparse
from array import array
import copy
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import SetBool
from tf2_ros import StaticTransformBroadcaster
from uav_inference_interfaces.srv import InferDetections
from uav_object_detection_interfaces.msg import ObjectDetectionFrame
from uav_object_detection_interfaces.srv import DetectObjects


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    image=cv2.imread(str(args.image))
    if image is None:raise ValueError('Cannot read recorded RGB image')
    h,w=image.shape[:2]
    rclpy.init();node=rclpy.create_node('object_detection_independent_probe',namespace='probe')
    results=[]

    def wait_for(condition,seconds=30.):
        deadline=time.monotonic()+seconds
        while not condition():
            if time.monotonic()>deadline:raise TimeoutError('ROS condition not reached')
            rclpy.spin_once(node,timeout_sec=.02)

    def call(client,request):
        wait_for(client.service_is_ready)
        future=client.call_async(request)
        wait_for(future.done,10.)
        result=future.result()
        if result is None:raise RuntimeError('No ROS response')
        return result

    try:
        inference=node.create_client(InferDetections,'inference/yolo/infer')
        tool=node.create_client(DetectObjects,'tools/object_detection/detect')
        enable=node.create_client(SetBool,'tools/object_detection/enable')
        rgbs=node.create_publisher(Image,'sensor_measurements/FrontCamera/rgb/image',qos_profile_sensor_data)
        depths=node.create_publisher(Image,'sensor_measurements/FrontCamera/depth_planar/image',qos_profile_sensor_data)
        ri=node.create_publisher(CameraInfo,'sensor_measurements/FrontCamera/rgb/camera_info',qos_profile_sensor_data)
        di=node.create_publisher(CameraInfo,'sensor_measurements/FrontCamera/depth_planar/camera_info',qos_profile_sensor_data)
        stream=[]
        node.create_subscription(ObjectDetectionFrame,'perception/object_detection/detections',stream.append,10)
        broadcaster=StaticTransformBroadcaster(node)
        t=TransformStamped();t.header.frame_id='probe/odom';t.child_frame_id='probe/camera_optical'
        t.transform.translation.x=1.;t.transform.translation.y=2.;t.transform.translation.z=3.
        t.transform.rotation.z=2**-.5;t.transform.rotation.w=2**-.5
        broadcaster.sendTransform(t)
        wait_for(lambda:all(p.get_subscription_count()>0 for p in (rgbs,depths,ri,di)))
        stamp=node.get_clock().now().to_msg()
        rgb=Image(width=w,height=h,encoding='bgr8',step=w*3,data=array('B',image.tobytes()))
        rgb.header.frame_id='probe/camera_optical';rgb.header.stamp=stamp
        depth=Image(header=copy.deepcopy(rgb.header),width=w,height=h,encoding='32FC1',step=w*4,
                    data=array('B',np.full((h,w),2.,np.float32).tobytes()))
        info=CameraInfo(header=copy.deepcopy(rgb.header),width=w,height=h,
            k=[w/2,0.,(w-1)/2,0.,w/2,(h-1)/2,0.,0.,1.],d=[0.]*5)
        ri.publish(info);di.publish(info);depths.publish(depth)
        rgbs.publish(rgb)
        wait_for(lambda:any(m.valid and m.detections for m in stream))
        observed=next(m for m in stream if m.valid and m.detections)
        assert observed.header.frame_id=='probe/odom' and observed.header.stamp==stamp
        assert observed.image_header==rgb.header and observed.depth_header==depth.header
        assert all(m.position_valid and m.position.z==5. for m in observed.detections)
        results.append('ROS RGB/depth/CameraInfo subscriptions -> real CUDA -> streaming 3D measurement')
        raw=call(inference,InferDetections.Request(request_id='independent-inference',generation=31,image=rgb))
        assert raw.success,raw.error
        assert raw.result.header==rgb.header and raw.result.generation==31
        assert raw.result.detections and all(m.mask.header==rgb.header for m in raw.result.detections)
        results.append('independent Inference service with original header and binary masks')
        request=DetectObjects.Request(request_id='independent-tool',generation=32,image=rgb)
        answer=call(tool,request)
        assert answer.success,answer.error
        assert answer.result.generation==32 and answer.result.header.stamp==stamp
        assert all(m.position_valid and m.position.z==5. for m in answer.result.detections)
        assert call(enable,SetBool.Request(data=False)).success
        stopped=call(tool,request);assert not stopped.success and 'STOPPED' in stopped.error
        assert call(enable,SetBool.Request(data=True)).success
        resumed=call(tool,request);assert resumed.success,resumed.error
        assert resumed.result.tool_generation!=answer.result.tool_generation
        results.append('Tool stop rejects calls; explicit resume changes generation and returns 3D')
        bad=copy.deepcopy(request);bad.image.header.stamp.sec-=1
        rejected=call(tool,bad)
        assert not rejected.success and 'synchronization' in rejected.error,rejected.error
        assert all(not d.position_valid for d in rejected.result.detections)
        results.append('unmatched depth never produces a valid position')
        report=dict(passed=True,scope='separate ROS client -> Inference/Tool -> CUDA worker; recorded RGB, synthetic depth/TF; no live sensor accuracy or flight claim',
            checks=results,image_sha256=hashlib.sha256(args.image.read_bytes()).hexdigest(),
            detections=len(answer.result.detections),positions=[[d.position.x,d.position.y,d.position.z] for d in answer.result.detections],
            source_stamp_ns=stamp.sec*1_000_000_000+stamp.nanosec,model_version=raw.result.model_version)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
    finally:
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
