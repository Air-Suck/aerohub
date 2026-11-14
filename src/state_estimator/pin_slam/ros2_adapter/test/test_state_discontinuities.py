import threading
from types import SimpleNamespace

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

from uav_pin_slam_ros2.contracts import PointCloudContract
from uav_pin_slam_ros2.node import PinSlamNode


def result(x, *, tracking_ok=True, loop_corrected=False, frame=0, neural_points=None):
    pose = np.eye(4)
    pose[0, 3] = x
    return SimpleNamespace(
        pose=pose,
        tracking_ok=tracking_ok,
        loop_corrected=loop_corrected,
        neural_points=neural_points,
        processed_frame=frame,
    )


def cloud(second):
    message = PointCloud2()
    message.header.frame_id = "drone0/Lidar"
    message.header.stamp.sec = second
    message.width = 10
    message.height = 1
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    return message


class Engine:
    def __init__(self, results):
        self.results = iter(results)
        self.publish_map_requests = []

    def process(self, _message, _map_max_points, *, publish_map=True):
        self.publish_map_requests.append(publish_map)
        return next(self.results)

class Harness:
    _on_cloud = PinSlamNode._on_cloud
    _stamp_ns = staticmethod(PinSlamNode._stamp_ns)

    def __init__(self, results):
        self._engine = Engine(results)
        self._map_max_points = 100
        self._map_publish_every_n_frames = 2
        self._processed_input_count = 0
        self._contract = PointCloudContract("drone0/Lidar")
        self._lock = threading.Lock()
        self._previous_pose = None
        self._previous_stamp_ns = None
        self.states = []
        self.statuses = []
        self.events = []

    def _publish_map(self, _message, points):
        if points is not None:
            self.events.append("map")

    def _publish_state(self, _message, pose, twist):
        self.states.append((pose.copy(), twist))
        self.events.append("state")

    def _status(self, level, message, **values):
        self.statuses.append((level, message, values))

    def get_logger(self):
        raise AssertionError("the tested paths must not raise a processing exception")


def test_loop_correction_resets_twist_baseline():
    harness = Harness([
        result(0.0, frame=0),
        result(100.0, loop_corrected=True, frame=1),
        result(101.0, frame=2),
    ])

    harness._on_cloud(cloud(1))
    harness._on_cloud(cloud(2))
    harness._on_cloud(cloud(3))

    assert len(harness.states) == 1
    np.testing.assert_allclose(harness.states[0][1].linear, [1.0, 0.0, 0.0])
    assert any(status[1] == "loop_corrected_twist_reset" for status in harness.statuses)


def test_tracking_recovery_requires_new_consecutive_samples():
    harness = Harness([
        result(0.0, frame=0),
        result(50.0, tracking_ok=False, frame=1),
        result(100.0, frame=2),
        result(101.0, frame=3),
    ])

    harness._on_cloud(cloud(1))
    harness._on_cloud(cloud(2))
    harness._on_cloud(cloud(3))
    harness._on_cloud(cloud(4))

    assert len(harness.states) == 1
    np.testing.assert_allclose(harness.states[0][1].linear, [1.0, 0.0, 0.0])
    assert any(status[1] == "tracking_lost" for status in harness.statuses)


def test_map_snapshot_is_requested_at_configured_interval():
    harness = Harness([
        result(0.0, frame=0, neural_points=np.ones((1, 3))),
        result(1.0, frame=1),
        result(2.0, frame=2, neural_points=np.ones((1, 3))),
    ])

    harness._on_cloud(cloud(1))
    harness._on_cloud(cloud(2))
    harness._on_cloud(cloud(3))

    assert harness._engine.publish_map_requests == [True, False, True]
    assert harness.events == ["state", "state", "map"]
