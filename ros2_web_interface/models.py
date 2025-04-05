from pydantic import BaseModel, Field
from typing import Dict, Any, List

class GetDataQuery(BaseModel):
    topic: str = Field(..., description="ROS topic name", example="/ros2_web_service_test/chatter")
    timeout: float = Field(5.0, gt=0, description="Timeout in seconds", example=5.0)

class CallServiceRequest(BaseModel):
    payload: Dict[str, Any] = Field(
        default_factory=dict,
        description="Optional dictionary of fields for the ROS service request",
        example={"dock_id": 520}
    )

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