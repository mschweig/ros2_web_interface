# ros2_web_interface/main.py
from fastapi import FastAPI
from ros2_web_interface.api.registry import get_router
import rclpy
from rclpy.node import Node
import threading
import uvicorn

app = FastAPI()
ros_node: Node = None

@app.on_event("startup")
def startup():
    global ros_node
    rclpy.init()
    ros_node = ROSNode()
    threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True).start()
    app.include_router(get_router(ros_node))

class ROSNode(Node):
    def __init__(self):
        super().__init__('web_interface_node')


def main():
    uvicorn.run("ros2_web_interface.main:app", host="0.0.0.0", port=8000)

if __name__ == "__main__":
    main()