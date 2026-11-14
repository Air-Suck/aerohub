"""Sample SUPER's committed (primary prefix + backup) curve without retiming it."""
import bisect
import math


class InsufficientCoverage(ValueError):
    """A valid source plan has no executable future suffix; never retime it."""


class Polynomial:
    def __init__(self, start, durations, axes, order):
        self.start = float(start)
        self.durations = tuple(durations)
        if not math.isfinite(self.start) or self.start <= 0 or not 1 <= order <= 9:
            raise ValueError('invalid polynomial epoch/order')
        if not self.durations or len(self.durations) > 256 or any(not math.isfinite(t) or t <= 0 for t in self.durations):
            raise ValueError('invalid piece durations')
        size = (order+1)*len(self.durations)
        if len(axes) != 3 or any(len(a) != size or not all(math.isfinite(v) for v in a) for a in axes):
            raise ValueError('invalid coefficients')
        self.order = order
        self.axes = axes
        self.ends = []
        for duration in self.durations:
            self.ends.append((self.ends[-1] if self.ends else 0)+duration)
        self.end = self.start+self.ends[-1]
        if not math.isfinite(self.end):
            raise ValueError('nonfinite coverage')

    def sample(self, absolute):
        t = absolute-self.start
        if not math.isfinite(t) or t < -1e-6 or t > self.ends[-1]+1e-6:
            raise ValueError('sample outside original coverage')
        t = min(max(t,0),self.ends[-1])
        index = min(bisect.bisect_right(self.ends,t),len(self.ends)-1)
        local = t-(self.ends[index-1] if index else 0)
        values = []
        for derivative in range(3):
            vector = []
            for axis in self.axes:
                coefficients = axis[index*(self.order+1):(index+1)*(self.order+1)]
                for _ in range(derivative):
                    degree = len(coefficients)-1
                    coefficients = [c*(degree-i) for i,c in enumerate(coefficients[:-1])]
                value = 0.0
                for coefficient in coefficients:
                    value = value*local+coefficient
                vector.append(value)
            values.append(vector)
        return values

    def times(self, start, step=.02):
        if start < self.start or self.end-start < .02 or not math.isfinite(step) or step <= 0:
            raise InsufficientCoverage('insufficient future coverage')
        count = math.ceil((self.end-start)/step)
        if count > 10000:
            raise ValueError('excessive trajectory')
        # ROS durations have nanosecond resolution. A grid sample and a source
        # knot can differ as floats yet serialize to the same duration. Keep
        # the original knot (including backup entry) in preference to the grid.
        by_stamp = {round((i*step)*1e9): start+i*step for i in range(1,count)}
        for offset in self.ends:
            absolute = self.start+offset
            if start < absolute < self.end:
                by_stamp[round((absolute-start)*1e9)] = absolute
        by_stamp[0] = start
        by_stamp[round((self.end-start)*1e9)] = self.end
        return [by_stamp[stamp] for stamp in sorted(by_stamp)]
