from fastapi import APIRouter, HTTPException, Body, Depends
from typing import Optional
from ros2_web_interface.ros.factory import ROSInterfaceFactory
from ros2_web_interface.models import CallServiceRequest, CallServiceResponse, ServiceQuery


def get_service_router(ros_node):
    router = APIRouter()

    @router.post("/call_service", response_model=CallServiceResponse)
    def call_service(
        query: ServiceQuery = Depends(),
        req: Optional[CallServiceRequest] = Body(default=None, description="Service request body (unwrapped)")
    ):
        handler = ROSInterfaceFactory.get_handler("service", ros_node)
        payload = req.dict() if req else {}
        try:
            return handler.call(query.topic, payload, query.timeout)
        except TimeoutError as e:
            raise HTTPException(status_code=504, detail=str(e))
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))

    return router