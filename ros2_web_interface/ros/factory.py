from ros2_web_interface.ros.topic import TopicHandler
from ros2_web_interface.ros.system import SystemHandler
from ros2_web_interface.ros.service import ServiceHandler
from ros2_web_interface.ros.action import ActionHandler

class ROSInterfaceFactory:
    @staticmethod
    def get_handler(kind: str, node):
        if kind == "topic":
            return TopicHandler(node)
        if kind == "system":
            return SystemHandler(node)
        if kind == "service":
            return ServiceHandler(node)
        if kind == "action":
            return ActionHandler(node)
        raise ValueError(f"Unsupported ROS interface kind: {kind}")