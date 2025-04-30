import threading
import rclpy
from abc import ABC, abstractmethod


class ROSInterface(ABC):
    _spin_lock = threading.Lock()

    def __init__(self, node):
        self.node = node

    def spin_until_future_complete(self, future, timeout=None):
        with ROSInterface._spin_lock:
            return rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

    def spin_once(self, timeout_sec=0.1):
        with ROSInterface._spin_lock:
            rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    @abstractmethod
    def call(self, name: str, *args, **kwargs):
        pass
