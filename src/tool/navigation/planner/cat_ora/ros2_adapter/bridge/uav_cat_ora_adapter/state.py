"""CAT-ORA input checks, independent of ROS runtime and algorithm numerics."""
import math


def checked_pose(message, frame, now, previous=0.0, max_age=1.0, max_future=.05):
    stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
    if message.header.frame_id!=frame:
        raise ValueError('unexpected frame: '+message.header.frame_id)
    if stamp<=0 or now<=0 or now-stamp>max_age or stamp-now>max_future:
        raise ValueError('pose timestamp outside input window')
    if stamp<=previous:
        raise ValueError('duplicate or out-of-order pose')
    pose=message.pose.pose if hasattr(message.pose,'pose') else message.pose
    p,q=pose.position,pose.orientation
    if not all(math.isfinite(x) for x in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)):
        raise ValueError('nonfinite pose')
    norm=sum(x*x for x in (q.x,q.y,q.z,q.w))
    if abs(norm-1)>.01:
        raise ValueError('invalid orientation')
    return stamp
