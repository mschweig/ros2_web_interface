from fastapi import APIRouter, HTTPException, Depends, Response
from ros2_web_interface.ros.factory import ROSInterfaceFactory
from ros2_web_interface.models import GetDataQuery, MessageResponse

def get_topic_router(ros_node):
    router = APIRouter()

    @router.get("/get_data", response_model=MessageResponse, responses={200: {"description": "JSON or image response"}})
    def get_data(query: GetDataQuery = Depends()):
        handler = ROSInterfaceFactory.get_handler("topic", ros_node)
        try:
            result = handler.call(query.topic, query.timeout)
            if isinstance(result, Response):
                return result
            return result
        except TimeoutError as e:
            raise HTTPException(status_code=504, detail=str(e))
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))

    return router