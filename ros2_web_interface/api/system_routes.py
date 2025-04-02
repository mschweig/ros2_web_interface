# ros2_web_interface/api/system_routes.py
from fastapi import APIRouter
from ros2_web_interface.ros.factory import ROSInterfaceFactory

def get_system_router(ros_node):
    router = APIRouter()

    @router.get("/list_topics")
    def list_topics():
        handler = ROSInterfaceFactory.get_handler("system", ros_node)
        return handler.call("list_topics")

    @router.get("/list_nodes")
    def list_nodes():
        handler = ROSInterfaceFactory.get_handler("system", ros_node)
        return handler.call("list_nodes")

    return router