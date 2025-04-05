from fastapi import APIRouter, HTTPException, Body, Query
from typing import Optional, Dict, Any
from ros2_web_interface.ros.factory import ROSInterfaceFactory
from ros2_web_interface.models import CallServiceResponse


def get_service_router(ros_node):
    router = APIRouter()

    @router.post("/call_service", response_model=CallServiceResponse)
    def call_service(
        topic: str = Query(..., description="Full ROS service topic name", example="/timon/dock"),
        timeout: float = Query(30.0, description="Timeout in seconds to wait for service response", example=20.0),
        body: Optional[Dict[str, Any]] = Body(default={}, description="Service request fields (unwrapped)")
    ):
        handler = ROSInterfaceFactory.get_handler("service", ros_node)
        payload = body or {}
        try:
            return handler.call(topic, payload, timeout)
        except TimeoutError as e:
            raise HTTPException(status_code=504, detail=str(e))
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))

    return router