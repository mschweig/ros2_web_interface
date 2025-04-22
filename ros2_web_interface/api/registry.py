from fastapi import APIRouter
from ros2_web_interface.api.system_routes import get_system_router
from ros2_web_interface.api.topic_routes import get_topic_router
from ros2_web_interface.api.service_routes import get_service_router
from ros2_web_interface.api.action_routes import get_action_router
from ros2_web_interface.api.action_routes import get_action_router


def get_router(ros_node):
    router = APIRouter()
    router.include_router(get_system_router(ros_node))
    router.include_router(get_topic_router(ros_node))
    router.include_router(get_service_router(ros_node))
    router.include_router(get_action_router(ros_node))
    return router