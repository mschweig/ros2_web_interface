import rclpy
import importlib
import time
from rclpy.action import ActionClient, get_action_names_and_types
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from ros2_web_interface.ros.base import ROSInterface


class ActionHandler(ROSInterface):
    def __init__(self, node: Node):
        super().__init__(node)

    def call(self, name: str, data=None, timeout=60.0):
        action_type_str = self._get_action_type(name)
        parts = action_type_str.split("/")
        if len(parts) == 3 and parts[1] == "action":
            pkg, _, act = parts
        else:
            pkg, act = parts

        module = importlib.import_module(f"{pkg}.action")
        action_class = getattr(module, act)

        client = ActionClient(self.node, action_class, name)
        if not client.wait_for_server(timeout_sec=5.0):
            raise TimeoutError(f"Action server {name} not available")

        goal_msg = action_class.Goal()
        if data:
            for k, v in data.items():
                setattr(goal_msg, k, v)

        send_goal_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            raise RuntimeError(f"Goal rejected by action server {name}")

        get_result_future = goal_handle.get_result_async()
        start_time = time.time()

        while not get_result_future.done():
            if time.time() - start_time > timeout:
                raise TimeoutError("Action call timed out")
            time.sleep(0.1)

        result = get_result_future.result().result
        return message_to_ordereddict(result)

    def list(self):
        action_names_and_types = get_action_names_and_types(self.node)
        return [name for name, _ in action_names_and_types]

    def _get_action_type(self, action_name):
        for name, types in get_action_names_and_types(self.node):
            if name == action_name:
                return types[0]
        raise ValueError(f"Action {action_name} not found")
