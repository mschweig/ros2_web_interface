from pydantic import BaseModel, Field
from typing import Dict, Any, List

class GetDataQuery(BaseModel):
    """Represents a query to get data from a ROS topic.

    Attributes:
        topic (str): The ROS topic name.
        timeout (float): Timeout in seconds for the query.
    """
    topic: str = Field(
        ..., 
        description="ROS topic name", 
        example="/ros2_web_service_test/chatter"
    )
    timeout: float = Field(
        10.0, 
        gt=0, 
        description="Timeout in seconds", 
        example=5.0
    )


class CallServiceRequest(BaseModel):
    """Represents a request to call a ROS service.

    Attributes:
        __root__ (Dict[str, Any]): Dictionary of service request fields.
    """
    __root__: Dict[str, Any] = Field(
        default_factory=dict,
        description="Dictionary of service request fields",
        example={"dock_id": 520}
    )

    def dict(self, *args, **kwargs) -> Dict[str, Any]:
        """Overrides the default dict method to return the root dictionary.

        Args:
            *args: Positional arguments for the parent dict method.
            **kwargs: Keyword arguments for the parent dict method.

        Returns:
            Dict[str, Any]: The root dictionary.
        """
        return super().dict(*args, **kwargs).get("__root__", {})


class ServiceQuery(BaseModel):
    """Represents a query to call a ROS service.

    Attributes:
        topic (str): The ROS service topic name.
        timeout (float): Timeout in seconds for the service call.
    """
    topic: str = Field(
        ..., 
        example="/robot/dock"
    )
    timeout: float = Field(
        30.0, 
        gt=0, 
        example=20.0
    )


class MessageResponse(BaseModel):
    """Represents a response message from a ROS topic.

    Attributes:
        topic (str): The ROS topic name.
        message_type (str): The type of the message.
        data (Dict[str, Any]): The message data.
    """
    topic: str = Field(
        ..., 
        example="/ros2_web_service_test/chatter"
    )
    message_type: str = Field(
        ..., 
        example="String"
    )
    data: Dict[str, Any] = Field(
        ..., 
        example={"data": "Hello from test!"}
    )


class CallServiceResponse(BaseModel):
    """Represents a response from a ROS service call.

    Attributes:
        success (bool): Whether the service call was successful.
        message (str): A message describing the result of the service call.
    """
    success: bool = Field(
        ..., 
        example=True
    )
    message: str = Field(
        ..., 
        example="Docking started."
    )


class TopicInfo(BaseModel):
    """Represents information about a ROS topic.

    Attributes:
        name (str): The name of the topic.
        types (List[str]): The message types associated with the topic.
    """
    name: str = Field(
        ..., 
        example="/chatter"
    )
    types: List[str] = Field(
        ..., 
        example=["std_msgs/msg/String"]
    )


class TopicListResponse(BaseModel):
    """Represents a response containing a list of ROS topics.

    Attributes:
        topics (List[TopicInfo]): A list of topic information.
    """
    topics: List[TopicInfo]


class NodeInfo(BaseModel):
    """Represents information about a ROS node.

    Attributes:
        name (str): The name of the node.
        namespace (str): The namespace of the node.
    """
    name: str = Field(
        ..., 
        example="/my_node"
    )
    namespace: str = Field(
        ..., 
        example="/"
    )


class NodeListResponse(BaseModel):
    """Represents a response containing a list of ROS nodes.

    Attributes:
        nodes (List[NodeInfo]): A list of node information.
    """
    nodes: List[NodeInfo]


class ServiceListResponse(BaseModel):
    """Represents a response containing a list of available ROS services.

    Attributes:
        services (List[str]): A list of available ROS service names.
    """
    services: List[str] = Field(
        ..., 
        description="List of available ROS services",
        example=["/robot/dock", "/robot/undock"]
    )