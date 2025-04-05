from fastapi import APIRouter
from ros2_web_interface.ros.factory import ROSInterfaceFactory
from ros2_web_interface.models import TopicListResponse, TopicInfo, NodeListResponse, NodeInfo

def get_system_router(ros_node):
    router = APIRouter()

    @router.get("/list_topics", response_model=TopicListResponse, responses={200: {"description": "List of available ROS topics with types"}})
    def list_topics():
        handler = ROSInterfaceFactory.get_handler("system", ros_node)
        raw = handler.call("list_topics")
        return {"topics": [TopicInfo(name=t[0], types=list(t[1])) for t in raw["topics"]]}

    @router.get("/list_nodes", response_model=NodeListResponse, responses={200: {"description": "List of active ROS nodes with namespaces"}})
    def list_nodes():
        handler = ROSInterfaceFactory.get_handler("system", ros_node)
        raw = handler.call("list_nodes")
        return {"nodes": [NodeInfo(name=n[0], namespace=n[1]) for n in raw["nodes"]]}

    return router