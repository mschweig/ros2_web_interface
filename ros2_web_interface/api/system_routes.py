from fastapi import APIRouter, HTTPException
from ros2_web_interface.ros.factory import ROSInterfaceFactory
from ros2_web_interface.models import TopicListResponse, NodeListResponse, ServiceListResponse


def get_system_router(ros_node):
    router = APIRouter()

    @router.get("/list_topics", response_model=TopicListResponse)
    def list_topics():
        handler = ROSInterfaceFactory.get_handler("system", ros_node)
        try:
            raw = handler.call("list_topics")
            return {"topics": [{"name": t[0], "types": list(t[1])} for t in raw["topics"]]}
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))

    @router.get("/list_nodes", response_model=NodeListResponse)
    def list_nodes():
        handler = ROSInterfaceFactory.get_handler("system", ros_node)
        try:
            raw = handler.call("list_nodes")
            return {"nodes": [{"name": n[0], "namespace": n[1]} for n in raw["nodes"]]}
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))

    @router.get("/list_services", response_model=ServiceListResponse)
    def list_services():
        handler = ROSInterfaceFactory.get_handler("system", ros_node)
        try:
            raw = handler.call("list_services")
            return {"services": raw}
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))

    return router
