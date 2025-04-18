from ros2_web_interface.ros.topic import TopicHandler
from ros2_web_interface.ros.system import SystemHandler
from ros2_web_interface.ros.service import ServiceHandler

class ROSInterfaceFactory:
    @staticmethod
    def get_handler(kind: str, node):
        if kind == "topic":
            return TopicHandler(node)
        if kind == "system":
            return SystemHandler(node)
        if kind == "service":
            return ServiceHandler(node)
        raise ValueError(f"Unsupported ROS interface kind: {kind}")