from pydantic import BaseModel, Field
from typing import Dict, Any, List, Optional

class GetDataQuery(BaseModel):
    topic: str = Field(..., description="ROS topic name", example="/ros2_web_service_test/chatter")
    timeout: float = Field(5.0, gt=0, description="Timeout in seconds", example=5.0)

class CallServiceRequest(BaseModel):
    __root__: Dict[str, Any] = Field(
        default_factory=dict,
        description="Dictionary of service request fields",
        example={"dock_id": 520}
    )

    def dict(self, *args, **kwargs):
        return super().dict(*args, **kwargs).get("__root__", {})

class ServiceQuery(BaseModel):
    topic: str = Field(..., example="/robot/dock")
    timeout: float = Field(30.0, gt=0, example=20.0)

class MessageResponse(BaseModel):
    topic: str = Field(..., example="/ros2_web_service_test/chatter")
    message_type: str = Field(..., example="String")
    data: Dict[str, Any] = Field(..., example={"data": "Hello from test!"})

class CallServiceResponse(BaseModel):
    success: bool = Field(..., example=True)
    message: str = Field(..., example="Docking started.")

class TopicInfo(BaseModel):
    name: str = Field(..., example="/chatter")
    types: List[str] = Field(..., example=["std_msgs/msg/String"])

class TopicListResponse(BaseModel):
    topics: List[TopicInfo]

class NodeInfo(BaseModel):
    name: str = Field(..., example="/my_node")
    namespace: str = Field(..., example="/")

class NodeListResponse(BaseModel):
    nodes: List[NodeInfo]

class ServiceListResponse(BaseModel):
    services: List[str] = Field(
        ..., 
        description="List of available ROS services",
        example=["/robot/dock", "/robot/undock"]
    )