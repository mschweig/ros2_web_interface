import threading
import time
from sensor_msgs.msg import Image as ROSImage
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
from cv_bridge import CvBridge
import cv2
from fastapi import Response
from ros2_web_interface.models import MessageResponse
from ros2_web_interface.ros.base import ROSInterface


class TopicHandler(ROSInterface):
    def __init__(self, node):
        super().__init__(node)
        self.lock = threading.Lock()
        self.event = threading.Event()
        self.latest_msg = None

    def call(self, name: str, timeout: float):
        msg_type = self._get_msg_type(name)
        msg_class = get_message(msg_type)

        with self.lock:
            self.latest_msg = None
            self.event.clear()
            sub = self.node.create_subscription(
                msg_class, name, self._callback, qos_profile=10
            )

            start = time.time()
            while not self.event.is_set():
                if time.time() - start > timeout:
                    self.node.destroy_subscription(sub)
                    raise TimeoutError(f"Timeout waiting for message on {name}")
                self.spin_once(timeout_sec=0.1)

            self.node.destroy_subscription(sub)
            return self._format_response(name, self.latest_msg)

    def _callback(self, msg):
        if self.latest_msg is None:
            self.latest_msg = msg
            self.event.set()

    def _get_msg_type(self, topic):
        for name, types in self.node.get_topic_names_and_types():
            if name == topic:
                return types[0]
        raise ValueError(f"Topic {topic} not found")

    def _format_response(self, topic, msg):
        if isinstance(msg, ROSImage):
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            success, buffer = cv2.imencode(".png", cv_image)
            if not success:
                raise RuntimeError("Failed to encode image")
            return Response(content=buffer.tobytes(), media_type="image/png")
        else:
            return MessageResponse(
                topic=topic,
                message_type=msg.__class__.__name__,
                data=message_to_ordereddict(msg)
            )
