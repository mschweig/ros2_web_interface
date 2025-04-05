from ros2_web_interface.ros.base import ROSInterface

class SystemHandler(ROSInterface):
    def __init__(self, node):
        self.node = node

    def call(self, name: str, data=None):
        if name == "list_topics":
            return {"topics": self.node.get_topic_names_and_types()}
        elif name == "list_nodes":
            return {"nodes": self.node.get_node_names_and_namespaces()}
        else:
            raise ValueError(f"Unknown system call: {name}")