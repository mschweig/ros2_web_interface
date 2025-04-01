from fastapi import FastAPI, Depends, HTTPException
from typing import Union
import rclpy
from rclpy.node import Node
import threading
import uvicorn
from ros2_web_interface.models import TopicQuery, ImageResponse, MessageResponse
from ros2_web_interface.ros.factory import ROSInterfaceFactory

app = FastAPI()
ros_node: Node = None

@app.on_event("startup")
def startup():
    global ros_node
    rclpy.init()
    ros_node = ROSNode()
    threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True).start()

@app.get("/get_data", response_model=Union[ImageResponse, MessageResponse])
def get_data(query: TopicQuery = Depends()):
    handler = ROSInterfaceFactory.get_handler("topic", ros_node)
    try:
        return handler.call(query.topic, query.timeout)
    except TimeoutError as e:
        raise HTTPException(status_code=504, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/list_topics")
def list_topics():
    handler = ROSInterfaceFactory.get_handler("system", ros_node)
    return handler.call("list_topics")

@app.get("/list_nodes")
def list_topics():
    handler = ROSInterfaceFactory.get_handler("system", ros_node)
    return handler.call("list_nodes")

class ROSNode(Node):
    def __init__(self):
        super().__init__('web_interface_node')


def main():
    uvicorn.run("ros2_web_interface.main:app", host="0.0.0.0", port=8000)

if __name__ == "__main__":
    main()