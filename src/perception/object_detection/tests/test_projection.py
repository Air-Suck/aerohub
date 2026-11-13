"""Actual ROS message/TF2 tests; isolated software geometry, no accuracy claim."""
import copy
import unittest
from array import array

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_ros import Buffer
from uav_inference_interfaces.msg import Detection, DetectionFrame
from uav_object_detection.projection import Projector, depth_array


class ProjectionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()

    @classmethod
    def tearDownClass(cls):rclpy.shutdown()

    def setUp(self):
        self.header=Header(frame_id='camera_optical');self.header.stamp.sec=100
        self.buffer=Buffer()
        for sec,x in [(99,1.),(101,3.)]:
            t=TransformStamped();t.header.frame_id='odom';t.header.stamp.sec=sec
            t.child_frame_id='camera_optical';t.transform.translation.x=x
            t.transform.rotation.w=1.
            self.buffer.set_transform(t,'test')
        self.p=Projector(self.buffer,'odom',registered=True)
        self.info=CameraInfo(header=self.header,width=20,height=20,k=[10.,0.,10.,0.,10.,10.,0.,0.,1.],d=[0.]*5)
        self.p.on_info('rgb',self.info);self.p.on_info('depth',self.info)
        depth=np.full((20,20),2.,np.float32)
        self.image=Image(header=self.header,width=20,height=20,encoding='32FC1',step=80,data=depth.tobytes())
        self.p.on_depth(self.image)
        mask=np.ones((20,20),np.uint8)
        d=Detection(xyxy=[0.,0.,20.,20.],category='person',confidence=.9,
            mask=Image(header=self.header,width=20,height=20,encoding='mono8',step=20,data=mask.tobytes()))
        self.frame=DetectionFrame(header=self.header,request_id='geometry',generation=1,model_version='fixture',
            width=20,height=20,valid=True,detections=[d],timings_json='{}')

    def test_depth_stamp_historical_tf_interpolation_and_measurement(self):
        result=self.p.project(self.frame,8,lambda _:True)
        self.assertTrue(result.valid,result.error)
        self.assertEqual(result.header.stamp,self.header.stamp)
        self.assertEqual(result.image_header,self.header)
        self.assertEqual(result.header.frame_id,'odom')
        d=result.detections[0]
        self.assertEqual(d.depth_m,2.)
        self.assertEqual(d.position.z,2.)
        # At t=100 the TF x is 2; taking latest t=101 would add a whole meter.
        self.assertLess(abs(d.position.x-2.),.21)
        self.assertFalse(hasattr(d,'velocity'))

    def test_missing_tf_and_cancellation_never_return_valid_position(self):
        self.buffer.clear();self.p.wait=.001
        self.assertFalse(self.p.project(self.frame,8,lambda _:True).valid)
        result=self.p.project(self.frame,8,lambda _:False)
        self.assertIn('CANCELLED_GENERATION',result.error)
        self.assertFalse(result.detections[0].position_valid)

    def test_invalid_calibration_invalidates_old_calibration(self):
        bad=copy.deepcopy(self.info);bad.d=[.1,0.,0.,0.,0.]
        with self.assertRaises(ValueError):self.p.on_info('depth',bad)
        self.assertFalse(self.p.project(self.frame,8,lambda _:True).valid)

    def test_skew_and_invalid_depth_rejected(self):
        frame=copy.deepcopy(self.frame);frame.header.stamp.sec=102
        self.assertFalse(self.p.project(frame,8,lambda _:True).valid)
        self.p.depth.clear();bad=copy.deepcopy(self.image)
        bad.data=array('B',np.zeros((20,20),np.float32).tobytes());self.p.on_depth(bad)
        self.assertFalse(self.p.project(self.frame,8,lambda _:True).detections[0].position_valid)

    def test_16u_millimeters_big_endian_and_padded_stride(self):
        msg=Image(width=2,height=1,encoding='16UC1',is_bigendian=1,step=6,
                  data=np.array([1234,2000,0],dtype='>u2').tobytes())
        np.testing.assert_allclose(depth_array(msg,.001),[[1.234,2.]])


if __name__=='__main__':unittest.main()
