"""One blocking acquisition worker per physical sensor, without a frame backlog."""
from threading import Event, Thread


class SensorWorker(Thread):
    def __init__(self, name, callback):
        super().__init__(name=name)
        self.callback = callback
        self.pending = Event()
        self.stopping = Event()

    def request(self):
        # Timer ticks coalesce into at most one pending acquisition.
        self.pending.set()

    def stop(self):
        self.stopping.set()
        self.pending.set()

    def run(self):
        while True:
            self.pending.wait()
            self.pending.clear()
            if self.stopping.is_set():
                return
            self.callback()
