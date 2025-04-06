import rclpy
import importlib
import time
from rclpy.node import Node
from rclpy.task import Future
from rosidl_runtime_py.utilities import get_service
from ros2_web_interface.ros.base import ROSInterface
from rosidl_runtime_py.convert import message_to_ordereddict


class ServiceHandler(ROSInterface):
    def __init__(self, node: Node):
        super().__init__(node)

    def call(self, name: str, data=None, timeout=30.0):
        srv_type_str = self._get_service_type(name)
        parts = srv_type_str.split("/")
        if len(parts) == 3 and parts[1] == "srv":
            pkg, _, srv = parts
        else:
            pkg, srv = parts

        module = importlib.import_module(f"{pkg}.srv")
        srv_class = getattr(module, srv)

        client = self.node.create_client(srv_class, name)
        if not client.wait_for_service(timeout_sec=5.0):
            raise TimeoutError(f"Service {name} not available")

        request = srv_class.Request()
        if data:
            for k, v in data.items():
                setattr(request, k, v)

        future: Future = client.call_async(request)
        start_time = time.time()

        while not future.done():
            if time.time() - start_time > timeout:
                self.node.destroy_client(client)
                raise TimeoutError("Service call timed out")
            time.sleep(0.1)

        result = future.result()
        self.node.destroy_client(client)

        return message_to_ordereddict(result)

    def _get_service_type(self, srv_name):
        for name, types in self.node.get_service_names_and_types():
            if name == srv_name:
                return types[0]
        raise ValueError(f"Service {srv_name} not found")