import threading
import rclpy
from abc import ABC, abstractmethod


class ROSInterface(ABC):
    _executor = None
    _spin_thread = None
    _spin_lock = threading.Lock()

    def __init__(self, node):
        self.node = node
        self._ensure_spin_thread()

    def _ensure_spin_thread(self):
        with self._spin_lock:
            if ROSInterface._executor is None:
                ROSInterface._executor = rclpy.executors.SingleThreadedExecutor()
                ROSInterface._executor.add_node(self.node)

            if ROSInterface._spin_thread is None:
                ROSInterface._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
                ROSInterface._spin_thread.start()

    def _spin_loop(self):
        rclpy.spin(self.node, executor=self._executor)

    @abstractmethod
    def call(self, name: str, *args, **kwargs):
        pass
