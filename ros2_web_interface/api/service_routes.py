from fastapi import APIRouter, HTTPException, Body
from ros2_web_interface.ros.factory import ROSInterfaceFactory

def get_service_router(ros_node):
    router = APIRouter()

    @router.post("/call_service")
    def call_service(topic: str, float = 30.0, payload: dict = Body(default={})):
        handler = ROSInterfaceFactory.get_handler("service", ros_node)
        try:
            result = handler.call(topic, payload)
            return result  # Automatically converted to JSON
        except TimeoutError as e:
            raise HTTPException(status_code=504, detail=str(e))
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))

    return router