from fastapi import APIRouter, HTTPException, Body, Depends
from typing import Optional
from ros2_web_interface.ros.factory import ROSInterfaceFactory
from ros2_web_interface.models import (
    CallActionRequest, ActionQuery
)


def get_action_router(ros_node):
    router = APIRouter()

    @router.post("/call_action")
    def call_action(
        query: ActionQuery = Depends(),
        req: Optional[CallActionRequest] = Body(default=None, description="Action goal fields (unwrapped)")
    ):
        handler = ROSInterfaceFactory.get_handler("action", ros_node)
        payload = req.dict() if req else {}
        try:
            result = handler.call(query.topic, payload, query.timeout)
            return result
        except TimeoutError as e:
            raise HTTPException(status_code=504, detail=str(e))
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))

    @router.get("/list_actions")
    def list_actions():
        handler = ROSInterfaceFactory.get_handler("action", ros_node)
        try:
            return {"actions": handler.list()}
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))

    @router.get("/action_result")
    def action_result(goal_id: str):
        handler = ROSInterfaceFactory.get_handler("action", ros_node)
        try:
            return handler.get_result(goal_id)
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))

    return router
