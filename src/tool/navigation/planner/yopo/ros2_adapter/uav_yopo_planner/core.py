"""Reference-mode planning math and acknowledged timeline; no ROS/model imports."""
from dataclasses import dataclass
import math
import numpy as np


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def calculate_yaw(velocity, goal, previous, dt):
    # YOPO MIT: policy/poly_solver.py, calculate_yaw (bd8a7f91953c).
    direction = velocity / (np.linalg.norm(velocity) + 1e-5)
    distance = np.linalg.norm(goal)
    goal = goal / (distance + 1e-5)
    weight = 6 * abs(wrap(math.atan2(goal[1], goal[0]) - previous)) / math.pi
    desired = direction + weight * goal
    target = math.atan2(desired[1], desired[0]) if distance > .5 else previous
    change = np.clip(wrap(target - previous), -.5 * math.pi * dt, .5 * math.pi * dt)
    return wrap(previous + change)


@dataclass
class Curve:
    start: float
    duration: float
    coefficients: np.ndarray
    times: np.ndarray
    yaws: np.ndarray

    @classmethod
    def create(cls, start, initial, terminal, duration, yaw, goal):
        initial, terminal = np.asarray(initial), np.asarray(terminal)
        if (initial.shape != (3, 3) or terminal.shape != (3, 3) or
                not np.isfinite([initial, terminal]).all() or
                not math.isfinite(duration) or duration <= 0):
            raise ValueError("finite PVA and positive duration required")
        # ROS Duration has nanosecond resolution. Quantize once before building the
        # curve; 5.000000000000001 must not produce duplicate points at 5 seconds.
        duration_ns = round(duration * 1e9)
        if duration_ns < 1:
            raise ValueError("duration must cover at least one nanosecond")
        duration = duration_ns / 1e9
        # Coefficients in normalized time; rows are P, V, A, columns XYZ.
        c = np.zeros((6, 3))
        c[0], c[1], c[2] = initial[0], duration * initial[1], duration**2 * initial[2] / 2
        d = terminal[0] - c[0] - c[1] - c[2]
        v = duration * terminal[1] - c[1] - 2*c[2]
        a = duration**2 * terminal[2] - 2*c[2]
        c[3], c[4], c[5] = 10*d-4*v+a/2, -15*d+7*v-a, 6*d-3*v+a/2
        times = np.r_[np.arange(0, duration_ns, 20000000, dtype=np.int64), duration_ns] / 1e9
        curve = cls(start, duration, c, times, np.zeros(len(times)))
        curve.yaws[0] = yaw
        for i, t in enumerate(times[1:], 1):
            pva, _ = curve.sample(start + t)
            curve.yaws[i] = calculate_yaw(pva[1], goal-pva[0], curve.yaws[i-1], t-times[i-1])
        return curve

    @property
    def end(self):
        return self.start + self.duration

    def sample(self, epoch):
        t = np.clip((epoch - self.start) / self.duration, 0., 1.)
        c = self.coefficients
        p = sum(c[k]*t**k for k in range(6))
        v = sum(k*c[k]*t**(k-1) for k in range(1, 6)) / self.duration
        a = sum(k*(k-1)*c[k]*t**(k-2) for k in range(2, 6)) / self.duration**2
        elapsed = t * self.duration
        i = min(max(np.searchsorted(self.times, elapsed, side='right')-1, 0), len(self.times)-2)
        u = (elapsed-self.times[i]) / (self.times[i+1]-self.times[i])
        yaw = self.yaws[i] + u * wrap(self.yaws[i+1]-self.yaws[i])
        return np.array([p, v, a]), wrap(yaw)


class Timeline:
    """A rejected or unacknowledged update never changes accepted references."""
    def __init__(self):
        self.plans = {}
        self.pending = None
        self.revision = 0
        self.effective = 0

    @property
    def latest(self):
        return self.plans.get(self.revision)

    def propose(self, curve):
        if self.pending is not None:
            raise ValueError("execution request already pending")
        revision = self.revision + 1
        self.pending = revision, curve
        return revision

    def acknowledge(self, accepted):
        if self.pending is None:
            raise ValueError("no pending submission")
        revision, curve = self.pending
        self.pending = None
        if accepted:
            self.plans[revision] = curve
            self.revision = revision
        return revision

    def feedback(self, revision, elapsed, now, waiting):
        curve = self.plans.get(revision)
        if curve is None or revision < self.effective:
            return False
        # A feedback sample can arrive after transport delay, never before its epoch.
        epoch = curve.start + elapsed
        if not math.isfinite(elapsed) or elapsed < 0 or epoch > now + .02 or now-epoch > .5:
            return False
        if not waiting and elapsed > .12 and now > curve.start + .12:
            self.effective = revision
            self.plans = {r: p for r, p in self.plans.items() if r >= revision}
        return True

    def reference(self, epoch):
        if self.latest is None or not self.latest.start < epoch < self.latest.end:
            raise ValueError("handoff outside accepted coverage")
        return self.latest.sample(epoch)
