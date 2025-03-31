import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge
import yaml


class ConfigurableTestPublisherNode(Node):
    def __init__(self):
        super().__init__('test_publisher_node')
        self.bridge = CvBridge()
        self.chatter_count = 0

        # Get 'topics' param as a YAML string
        self.declare_parameter('topics', "")
        raw_param = self.get_parameter('topics').get_parameter_value().string_value

        if not raw_param:
            self.get_logger().error("No 'topics' parameter provided.")
            raise RuntimeError("Missing 'topics' parameter.")

        # Parse YAML string to dict
        self.config = yaml.safe_load(raw_param)
        self.topic_publishers = {}  # ✅ Renamed to avoid conflict
        self.setup_publishers()

        # Start publishing loop
        self.timer = self.create_timer(0.5, self.publish_all)

    def setup_publishers(self):
        for topic_cfg in self.config.values():
            topic_name = topic_cfg['name']
            topic_type = topic_cfg['type']

            if topic_type == 'message':
                pub = self.create_publisher(String, topic_name, 10)
                test_message = topic_cfg.get('test_message', 'Hello from test!')
                self.topic_publishers[topic_name] = {
                    'type': 'message',
                    'publisher': pub,
                    'test_message': test_message
                }
                self.get_logger().info(f"Configured String publisher for {topic_name}")

            elif topic_type == 'image':
                pub = self.create_publisher(Image, topic_name, 10)
                self.topic_publishers[topic_name] = {
                    'type': 'image',
                    'publisher': pub
                }
                self.get_logger().info(f"Configured Image publisher for {topic_name}")

            else:
                self.get_logger().warn(f"Unsupported type in config: {topic_type}")

    def publish_all(self):
        for topic_name, pub_info in self.topic_publishers.items():
            pub = pub_info['publisher']

            if pub_info['type'] == 'message':
                msg = String()
                msg.data = f"{pub_info['test_message']} Count: {self.chatter_count}"
                pub.publish(msg)

            elif pub_info['type'] == 'image':
                img = np.zeros((240, 320, 3), dtype=np.uint8)
                cv2.putText(img, "Test", (100, 120), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (255, 255, 255), 2)
                ros_img = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
                pub.publish(ros_img)

        self.chatter_count += 1
        if self.chatter_count % 5 == 0:
            self.get_logger().info(f"Published {self.chatter_count} message cycles.")


def main(args=None):
    rclpy.init(args=args)
    node = ConfigurableTestPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
