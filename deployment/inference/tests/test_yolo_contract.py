import copy
import unittest

import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from uav_inference_interfaces.msg import Detection, DetectionFrame
from uav_inference_interfaces.srv import InferDetections
from uav_inference.yolo_contract import array_image, validate_frame, validate_request


class YoloContractTest(unittest.TestCase):
    def setUp(self):
        header=Header(frame_id='camera_optical');header.stamp.sec=123;header.stamp.nanosec=456
        self.request=InferDetections.Request(request_id='contract',generation=7,
            image=Image(header=header,width=2,height=1,encoding='rgb8',step=8,
                        data=bytes([1,2,3,4,5,6,99,99])))
        self.frame=DetectionFrame(header=header,request_id='contract',generation=7,model_version='fixture',
            width=2,height=1,valid=True,detections=[Detection(xyxy=[0.,0.,2.,1.],confidence=.9,category='person',
                mask=Image(header=header,width=2,height=1,encoding='mono8',step=2,data=bytes([0,1])))])

    def test_rgb_stride_and_channel_order(self):
        np.testing.assert_array_equal(validate_request(self.request),[[[3,2,1],[6,5,4]]])
        broken=copy.deepcopy(self.request.image);broken.step=5
        with self.assertRaises(ValueError):array_image(broken)

    def test_missing_stamp_and_identity_rejected(self):
        bad=copy.deepcopy(self.request);bad.image.header.stamp.sec=bad.image.header.stamp.nanosec=0
        with self.assertRaises(ValueError):validate_request(bad)
        bad=copy.deepcopy(self.request);bad.request_id=''
        with self.assertRaises(ValueError):validate_request(bad)

    def test_ros_serialization_preserves_mask_and_identity(self):
        received=deserialize_message(serialize_message(self.frame),DetectionFrame)
        validate_frame(received,self.request)
        self.assertEqual(list(received.detections[0].mask.data),[0,1])
        received.generation+=1
        with self.assertRaises(ValueError):validate_frame(received,self.request)

    def test_corrupt_mask_and_geometry_rejected(self):
        for change in ('stamp','pixels','box'):
            frame=copy.deepcopy(self.frame)
            if change=='stamp':frame.detections[0].mask.header.stamp.nanosec+=1
            elif change=='pixels':frame.detections[0].mask.data=bytes([0,2])
            else:frame.detections[0].xyxy=[0.,0.,3.,1.]
            with self.assertRaises(ValueError):validate_frame(frame,self.request)


if __name__=='__main__':unittest.main()
