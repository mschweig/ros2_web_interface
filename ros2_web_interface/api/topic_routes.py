# ros2_web_interface/api/topic_routes.py
from fastapi import APIRouter, Depends, HTTPException, Response
from ros2_web_interface.models import TopicQuery, MessageResponse
from ros2_web_interface.ros.factory import ROSInterfaceFactory

def get_topic_router(ros_node):
    router = APIRouter()

    @router.get("/get_data", response_class=Response)
    def get_data(query: TopicQuery = Depends()):
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