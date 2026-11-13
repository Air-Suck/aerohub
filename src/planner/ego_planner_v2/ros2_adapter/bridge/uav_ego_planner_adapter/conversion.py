"""Preserve absolute-time polynomial PVA through AeroHub's fleet NED contract."""
import math
import numpy as np
from numpy.polynomial import polynomial as polynomial
from uav_planner_interfaces.msg import TimedTrajectoryPoint, TimedTrajectorySegment, VehicleTrajectory

def nanoseconds(stamp):
    return stamp.sec*1_000_000_000 + stamp.nanosec

def set_ns(stamp, value):
    stamp.sec, stamp.nanosec = divmod(int(value), 1_000_000_000)

def peak(coefficients, duration, derivative):
    # Normalize time to [0,1] before finding stationary points of squared norm.
    c = coefficients[:, ::-1] * np.power(duration, np.arange(6))
    c = np.array([polynomial.polyder(axis, derivative) for axis in c]) / duration**derivative
    norm_squared = np.zeros(1)
    for axis in c: norm_squared = polynomial.polyadd(norm_squared, polynomial.polymul(axis, axis))
    try:
        roots = polynomial.polyroots(polynomial.polyder(norm_squared))
    except np.linalg.LinAlgError as exc:
        raise ValueError('cannot verify polynomial extrema') from exc
    times = [0., 1.] + [float(r.real) for r in roots if abs(r.imag) < 1e-8 and 0 < r.real < 1]
    return max(float(np.linalg.norm([polynomial.polyval(t, axis) for axis in c])) for t in times)

class Polynomial:
    def __init__(self, msg, expected_id, max_velocity, max_acceleration):
        if msg.drone_id != expected_id or msg.order != 5 or not 1 <= len(msg.duration) <= 256:
            raise ValueError('wrong vehicle, polynomial order or piece count')
        self.start_ns = nanoseconds(msg.start_time)
        self.durations = np.asarray(msg.duration, dtype=float)
        if any(len(c) != 6*len(self.durations) for c in (msg.coef_x, msg.coef_y, msg.coef_z)):
            raise ValueError('coefficient count mismatch')
        self.coefficients = np.asarray([msg.coef_x, msg.coef_y, msg.coef_z], dtype=float).reshape(3,-1,6).transpose(1,0,2)
        if self.start_ns <= 0 or not np.isfinite(self.durations).all() or not np.isfinite(self.coefficients).all() or np.any(self.durations < .001):
            raise ValueError('nonfinite or invalid polynomial')
        self.ends = np.cumsum(self.durations)
        self.left_ends = {self.start_ns + round(float(end)*1e9): index
                          for index,end in enumerate(self.ends)}
        if self.ends[-1] > 600: raise ValueError('trajectory coverage exceeds ten minutes')
        self.end_ns = self.start_ns + round(float(self.ends[-1])*1e9)
        for coefficients, duration in zip(self.coefficients, self.durations):
            if peak(coefficients,duration,1) > max_velocity or peak(coefficients,duration,2) > max_acceleration:
                raise ValueError('infeasible polynomial')
        self.traj_id = msg.traj_id

    def sample(self, absolute_ns):
        if absolute_ns in self.left_ends:
            # ROS timestamps quantize the original float32 durations to ns.
            # Define exact knot samples as left limits consistently; otherwise
            # rounding can pick either side of a tiny wire discontinuity.
            index = self.left_ends[absolute_ns]
            coefficients, t = self.coefficients[index], self.durations[index]
            return [np.array([np.polyval(np.polyder(axis,d),t) for axis in coefficients]) for d in range(3)]
        elapsed = (absolute_ns-self.start_ns)/1e9
        if elapsed < -1e-8 or elapsed > self.ends[-1]+1e-8:
            raise ValueError('sample outside original trajectory coverage')
        index = min(int(np.searchsorted(self.ends, elapsed, side='right')), len(self.ends)-1)
        t = elapsed - (self.ends[index-1] if index else 0.)
        coefficients = self.coefficients[index]
        return [np.array([np.polyval(np.polyder(axis, d), t) for axis in coefficients]) for d in range(3)]

    def final(self, goal):
        p,v,a = self.sample(self.end_ns)
        return np.linalg.norm(p-goal) < .03 and np.linalg.norm(v) < 1e-4 and np.linalg.norm(a) < 1e-4

def equivalent_final_tail(previous, candidate, goal, now_ns, acceleration_tolerance):
    """Keep an accepted final path only if a late replacement is redundant.

    Check the full polynomial difference on every remaining piece, including
    stationary endpoint extensions. This does not extend any execution epoch.
    """
    if not previous.final(goal) or not candidate.final(goal): return False
    start_ns = max(previous.start_ns,candidate.start_ns)
    if now_ns < start_ns: return False
    # Results can arrive after both tails ended while the executor confirms
    # hover. Compare the ENTIRE shared tail, including its already elapsed
    # portion, rather than treating an empty remaining interval as equivalent.
    knots = {start_ns,previous.end_ns,candidate.end_ns}
    for source in [previous,candidate]:
        knots |= {source.start_ns+round(float(t)*1e9) for t in source.ends}
    knots = sorted(t for t in knots if t>=start_ns)
    def shifted(source,t):
        if t>=source.end_ns:
            c=np.zeros((3,6));c[:,-1]=source.sample(source.end_ns)[0];return c
        elapsed=(t-source.start_ns)/1e9
        index=min(int(np.searchsorted(source.ends,elapsed,side='right')),len(source.ends)-1)
        offset=elapsed-(source.ends[index-1] if index else 0.)
        return np.array([np.pad(np.poly1d(axis)(np.poly1d([1.,offset])).c,
                        (6-len(np.poly1d(axis)(np.poly1d([1.,offset])).c),0))
                        for axis in source.coefficients[index]])
    for left,right in zip(knots,knots[1:]):
        delta=shifted(previous,left)-shifted(candidate,left)
        for derivative,tolerance in enumerate([1e-4,1e-4,acceleration_tolerance]):
            if peak(delta,(right-left)/1e9,derivative)>tolerance:return False
    return True

def validate_terminal_hold(source, duration=.1):
    """Bound the Hermite extension to a stationary endpoint (including wire noise).

    Preserve the original endpoint PVA. A second knot at the same position with
    zero derivatives can otherwise amplify tiny nonzero float32 derivatives.
    Check the entire appended quintic, not just its two endpoints.
    """
    _,v,a = source.sample(source.end_ns)
    vh, ah2 = v*duration, a*duration*duration
    # Normalized quintic coefficients of displacement from the terminal position.
    c = np.array([-3*vh-.5*ah2, 8*vh+1.5*ah2, -6*vh-1.5*ah2,
                  .5*ah2, vh, np.zeros(3)]).T
    c = c / np.power(duration, np.arange(5,-1,-1))
    if any(peak(c,duration,d)>1e-4 for d in range(3)):
        raise ValueError('terminal hold is not stationary within PVA tolerance')


def segment_for(config, trajectories, goals, now_ns):
    parameters = config['planner']['parameters']
    start_ns = now_ns + round(parameters['valid_from_lead']*1e9)
    end_ns = min(t.end_ns for t in trajectories.values())
    # For a final fleet plan, extend completed vehicles with a stationary endpoint
    # only when endpoint PVA is already at rest. This keeps one common end epoch.
    final = all(trajectories[v].final(goals[v]) for v in config['vehicles'])
    if final:
        end_ns = max(t.end_ns for t in trajectories.values())
        # A final replan can be shorter than the normal dispatch lead. Crop at
        # an earlier future instant instead of moving the source polynomial.
        start_ns = min(start_ns, max(now_ns+50_000_000, end_ns-100_000_000))
        # A sub-millisecond moving suffix would introduce an ill-conditioned
        # Hermite interval, below the native polynomial's own 1 ms floor.
        if end_ns-start_ns < 1_000_000:
            raise ValueError('final trajectory lacks 50 ms dispatch budget plus 1 ms moving suffix')
        if end_ns-start_ns < 100_000_000:
            # The native motion still has a dispatchable future tail. Keep its
            # original epoch and every knot, and append a bounded rest interval
            # AFTER the last native endpoint. Never stretch the moving curve.
            # float32 native pieces have tiny unequal left/right knot values.
            # A single Hermite knot cannot represent both; a very short span
            # can amplify that wire discontinuity. Hand over on the right of
            # the final native piece boundary instead. The accepted old plan
            # remains active until this future instant; no source time changes.
            start_ns = max([start_ns] + [
                source.start_ns + round(float(source.ends[-2])*1e9) + 1
                for source in trajectories.values() if len(source.ends)>1])
            if end_ns-start_ns < 1_000_000:
                raise ValueError('final native piece has no dispatchable 1 ms moving suffix')
            end_ns += 100_000_000
            for source in trajectories.values():
                if source.end_ns >= start_ns:
                    validate_terminal_hold(source,(end_ns-source.end_ns)/1e9)
    if end_ns-start_ns < 100_000_000:
        raise ValueError('insufficient future coverage')
    segment = TimedTrajectorySegment(interface_version=1, task_id=config['task']['task_id'], is_final=final)
    set_ns(segment.generated_at, now_ns); set_ns(segment.valid_from, start_ns)
    for vehicle in config['vehicles']:
        source = trajectories[vehicle]
        if start_ns < source.start_ns or (start_ns >= source.end_ns and not final):
            raise ValueError('vehicle has no common future coverage')
        knots = {start_ns, end_ns, source.end_ns}
        knots |= {source.start_ns + round(float(t)*1e9) for t in source.ends}
        knots = sorted(t for t in knots if start_ns <= t <= end_ns)
        # Endpoint PVA reconstructs each degree-five polynomial exactly. Adding
        # uniform samples beside an original knot creates short Hermite spans
        # that amplify float32 wire discontinuities in the second derivative.
        # Keep every original knot and both crop endpoints, without retiming.
        times = knots
        item = VehicleTrajectory(vehicle_id=vehicle, valid_fields=3)
        for t in times:
            p,v,a = source.sample(min(t, source.end_ns))
            if t > source.end_ns:
                v, a = np.zeros(3), np.zeros(3)
            point = TimedTrajectoryPoint()
            set_ns(point.time_from_start, t-start_ns)
            for target, value in [(point.position,p),(point.velocity,v),(point.acceleration,a)]:
                target.x, target.y, target.z = float(value[1]), float(value[0]), -float(value[2])
            item.points.append(point)
        segment.vehicle_trajectories.append(item)
    return segment

def world_odometry(msg, vehicle, origin_ned):
    """Translate canonical local ENU pose; retain body-FLU twist and covariance.

    Native launch explicitly enables body -> world velocity at the FSM input.
    This avoids publishing a nonstandard Odometry with world twist/body pose.
    """
    import copy
    if msg.header.frame_id != vehicle+'/odom' or msg.child_frame_id != vehicle+'/base_link':
        raise ValueError('unsupported odometry frames')
    q = msg.pose.pose.orientation
    values = np.array([q.x,q.y,q.z,q.w],dtype=float)
    if not np.isfinite(values).all() or abs(np.linalg.norm(values)-1) > .001:
        raise ValueError('invalid quaternion')
    out = copy.deepcopy(msg)
    p = out.pose.pose.position
    p.x += origin_ned[1]; p.y += origin_ned[0]; p.z -= origin_ned[2]
    state = [p.x,p.y,p.z,out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.linear.z]
    if not all(math.isfinite(x) for x in state): raise ValueError('nonfinite odometry')
    out.header.frame_id = 'world'
    return out
