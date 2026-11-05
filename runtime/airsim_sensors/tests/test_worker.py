import threading
import unittest

from uav_as2_sensor_adapter.worker import SensorWorker


class SensorWorkerTests(unittest.TestCase):
    def test_blocked_camera_does_not_block_lidar(self):
        camera_started, release_camera, lidar_received = (threading.Event() for _ in range(3))

        def camera():
            camera_started.set()
            release_camera.wait(2)

        workers = [SensorWorker('camera', camera), SensorWorker('lidar', lidar_received.set)]
        for worker in workers:
            worker.start()
        try:
            workers[0].request()
            self.assertTrue(camera_started.wait(1))
            workers[1].request()
            self.assertTrue(lidar_received.wait(1))
        finally:
            release_camera.set()
            for worker in workers:
                worker.stop()
            for worker in workers:
                worker.join(2)
                self.assertFalse(worker.is_alive())

    def test_slow_sensor_coalesces_ticks_and_never_overlaps(self):
        started, release, twice = (threading.Event() for _ in range(3))
        calls = []

        def callback():
            calls.append(threading.get_ident())
            if len(calls) == 1:
                started.set()
                release.wait(2)
            else:
                twice.set()

        worker = SensorWorker('slow', callback)
        worker.start()
        try:
            worker.request()
            self.assertTrue(started.wait(1))
            for _ in range(100):
                worker.request()
            self.assertEqual(len(calls), 1)
            release.set()
            self.assertTrue(twice.wait(1))
        finally:
            release.set()
            worker.stop()
            worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(calls), 2)
        self.assertEqual(len(set(calls)), 1)

    def test_idle_worker_stops_without_an_acquisition(self):
        calls = []
        worker = SensorWorker('idle', lambda: calls.append(True))
        worker.start()
        worker.stop()
        worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(calls, [])
