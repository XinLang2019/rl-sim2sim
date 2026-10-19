"""A single replaceable raw frame; slow encoding never queues historical images."""
import threading


class LatestFrame:
    def __init__(self):
        self.condition = threading.Condition()
        self.pending = None

    def put(self, value):
        with self.condition:
            self.pending = value
            self.condition.notify()

    def take(self, timeout=.1):
        with self.condition:
            self.condition.wait_for(lambda: self.pending is not None, timeout)
            value, self.pending = self.pending, None
            return value

    def clear(self):
        with self.condition:
            self.pending = None
