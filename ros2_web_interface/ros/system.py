from ros2_web_interface.ros.base import ROSInterface

class SystemHandler(ROSInterface):
    def __init__(self, node):
        super().__init__(node)

    def call(self, name: str, data=None):
        if name == "list_topics":
            all_topics = self.node.get_topic_names_and_types()
            filtered = [(topic, types) for topic, types in all_topics if "/_action/" not in topic]
            return {"topics": filtered}

        elif name == "list_nodes":
            return {"nodes": self.node.get_node_names_and_namespaces()}

        elif name == "list_services":
            all_services = self.node.get_service_names_and_types()
            filtered = [srv for srv, _ in all_services if "/_action/" not in srv]
            return filtered
        else:
            raise ValueError(f"Unknown system call: {name}")