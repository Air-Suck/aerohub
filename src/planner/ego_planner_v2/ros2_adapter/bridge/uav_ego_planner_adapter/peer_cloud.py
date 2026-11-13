"""Keep known fleet body returns out of EGO's static occupancy map.

Input coordinates are fleet ENU; centers come from measured PX4 odometry,
never simulator truth. This is the original reproduction's 0.75 m body mask,
with explicit wall-time and sample-time freshness guards. It is not a general
dynamic-object classifier; nearby static surfaces inside the mask are omitted.
"""
import math


def remove_peer_returns(points, histories, cloud_ns, now_wall, radius):
    if not math.isfinite(radius) or not 0 < radius <= .75:
        raise ValueError('invalid peer body mask radius')
    centers=[]
    for history in histories:
        if not history or not 0 <= now_wall-history[-1][1] <= .4:
            raise ValueError('stale/missing measured peer odometry')
        stamp, received, position=min(history,key=lambda row:abs(row[0]-cloud_ns))
        if abs(stamp-cloud_ns)>200_000_000 or not 0 <= now_wall-received <= .4:
            raise ValueError('peer/cloud sample time mismatch')
        if not all(math.isfinite(v) for v in position):
            raise ValueError('invalid measured peer position')
        centers.append(position)
    kept=[p for p in points if all(math.dist(p,center)>radius for center in centers)]
    return kept,len(points)-len(kept)
