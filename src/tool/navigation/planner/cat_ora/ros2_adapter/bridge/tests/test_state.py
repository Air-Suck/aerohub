import unittest
from types import SimpleNamespace as S
from uav_cat_ora_adapter.state import checked_pose

class StateTest(unittest.TestCase):
    def pose(self,stamp=10.,frame='drone0/odom',x=1.):
        return S(header=S(frame_id=frame,stamp=S(sec=int(stamp),nanosec=int((stamp-int(stamp))*1e9))),
                 pose=S(position=S(x=x,y=2.,z=3.),orientation=S(x=0.,y=0.,z=0.,w=1.)))

    def test_fresh_local_pose(self):
        self.assertEqual(checked_pose(self.pose(),'drone0/odom',10.1),10.)

    def test_wrong_frame_stale_future_and_nonfinite(self):
        for m,now,last in [(self.pose(frame='earth'),10.,0),
                           (self.pose(),12.,0),(self.pose(),9.,0),
                           (self.pose(),10.,10.),(self.pose(x=float('nan')),10.,0)]:
            with self.subTest(message=m,now=now,last=last):
                with self.assertRaises(ValueError):checked_pose(m,'drone0/odom',now,last)

    def test_nonzero_origin_is_preserved_and_invalid_quaternion_rejected(self):
        m=self.pose(x=-123.)
        self.assertEqual(checked_pose(m,'drone0/odom',10.1),10.)
        self.assertEqual(m.pose.position.x,-123.)
        m.pose.orientation.w=0.
        with self.assertRaises(ValueError):checked_pose(m,'drone0/odom',10.1)

if __name__=='__main__':unittest.main()
