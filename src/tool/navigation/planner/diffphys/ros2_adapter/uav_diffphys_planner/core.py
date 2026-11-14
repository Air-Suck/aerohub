"""Official swarm observation and control decoding, in a fixed z-up world frame."""
import numpy as np
from scipy.spatial.transform import Rotation


def observation(position, rotation, velocity, goal, speed=2.5, margin=.15, full_attitude=False,
                goal_lookahead_m=0.):
    position, velocity, goal = map(lambda x: np.asarray(x, dtype=float), (position, velocity, goal))
    if any(x.shape != (3,) or not np.isfinite(x).all() for x in (position,velocity,goal)):
        raise ValueError('invalid position/velocity/goal')
    if np.shape(rotation)!=(3,3) or not np.isfinite(rotation).all(): raise ValueError('invalid rotation')
    forward=rotation[:,0].copy();forward[2]=0
    if np.linalg.norm(forward)<1e-6: raise ValueError('vertical heading')
    forward/=np.linalg.norm(forward)
    level=np.column_stack((forward,np.cross([0.,0.,1.],forward),[0.,0.,1.]))
    relative=goal-position;distance=np.linalg.norm(relative)
    if not np.isfinite(goal_lookahead_m) or goal_lookahead_m<0:
        raise ValueError('invalid goal lookahead')
    if goal_lookahead_m and np.linalg.norm(relative[:2])>goal_lookahead_m:
        relative=relative.copy()
        relative[:2]*=goal_lookahead_m/np.linalg.norm(relative[:2])
    steering_distance=np.linalg.norm(relative)
    # Lookahead changes steering direction, not the requested cruise speed.
    # Only the distance to the real goal may reduce speed near arrival.
    target_v=relative*(min(distance,speed)/max(steering_distance,1e-9))
    state=np.concatenate((velocity@level,target_v@level,rotation[2] if full_attitude else level[2], [margin]))
    return state,level


def decode(action, level, rotation, relative_goal, local_vertical_velocity, hover,
           max_thrust=15., gravity=9.80665, heading_vector=None, vertical_correction=.01,
           previous_forward=None, max_command_tilt_rad=0.):
    action=np.asarray(action,dtype=float)
    if action.shape!=(6,) or not np.isfinite(action).all(): raise ValueError('invalid action')
    pair=level@action.reshape(3,2)
    # ctl_error in official eval is synthetic random control error; deployment uses zero.
    acceleration=pair[:,0]-pair[:,1];acceleration[2]+=gravity
    norm=np.linalg.norm(acceleration)
    if norm<1e-6: raise ValueError('zero collective acceleration')
    up=acceleration/norm
    if up[2]<.5: raise ValueError('policy requested excessive tilt/inverted thrust')
    if not np.isfinite(max_command_tilt_rad) or not 0<=max_command_tilt_rad<np.pi/2:
        raise ValueError('invalid command tilt limit')
    if max_command_tilt_rad:
        horizontal=np.linalg.norm(acceleration[:2])
        limit=acceleration[2]*np.tan(max_command_tilt_rad)
        if horizontal>limit:
            acceleration[:2]*=limit/horizontal
            norm=np.linalg.norm(acceleration)
            up=acceleration/norm
    forward=(rotation[:,0] if previous_forward is None else np.asarray(previous_forward))*5+(relative_goal if heading_vector is None else heading_vector)
    forward[2]=-(forward[0]*up[0]+forward[1]*up[1])/up[2]
    forward/=np.linalg.norm(forward)
    left=np.cross(up,forward)
    desired=np.column_stack((forward,left,up))
    q=Rotation.from_matrix(desired).as_quat()
    throttle=(norm+local_vertical_velocity*abs(local_vertical_velocity)*vertical_correction)/9.8*hover
    if not np.isfinite(throttle) or not 0<throttle<=.85: raise ValueError('policy thrust outside deployment limits')
    return q,float(throttle*max_thrust)
