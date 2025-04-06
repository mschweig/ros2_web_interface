from ros2_web_interface.ros.base import ROSInterface


class SystemHandler(ROSInterface):
    def __init__(self, node):
        super().__init__(node)

    def call(self, name: str, data=None):
        if name == "list_topics":
            return {"topics": self.node.get_topic_names_and_types()}
        elif name == "list_nodes":
            return {"nodes": self.node.get_node_names_and_namespaces()}
        elif name == "list_services":
            return {"services": self.node.get_service_names_and_types()}
        else:
            raise ValueError(f"Unknown system call: {name}")
