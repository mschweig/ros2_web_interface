from pydantic import BaseModel, Field

class TopicQuery(BaseModel):
    topic: str = Field(..., description="ROS 2 topic to subscribe to")
    timeout: float = Field(5.0, gt=0.0, description="Timeout in seconds")

class ImageResponse(BaseModel):
    topic: str
    type: str = "image"
    format: str
    base64: str
    data_uri: str

class MessageResponse(BaseModel):
    topic: str
    type: str = "message"
    message_type: str
    data: dict