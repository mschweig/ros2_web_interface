import threading
import time
import rclpy
from abc import ABC, abstractmethod
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future

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
                ROSInterface._executor = SingleThreadedExecutor()
                ROSInterface._executor.add_node(self.node)

            if ROSInterface._spin_thread is None:
                ROSInterface._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
                ROSInterface._spin_thread.start()

    def _spin_loop(self):
        rclpy.spin(self.node, executor=self._executor)

    def spin_until_future_complete(self, future: Future, timeout: float = 30.0):
        start = time.time()
        while rclpy.ok():
            if future.done():
                return
            if time.time() - start > timeout:
                raise TimeoutError("Operation timed out")
            ROSInterface._executor.spin_once(timeout_sec=0.1)

    @abstractmethod
    def call(self, name: str, *args, **kwargs):
        pass
