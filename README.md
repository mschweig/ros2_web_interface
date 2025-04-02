# ROS 2 Web Interface

[![License](https://img.shields.io/github/license/mschweig/ros2_web_interface.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Python](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)

A modular, production-grade web interface for interacting with ROS 2 topics using FastAPI.

This project enables dynamic subscription to ROS 2 topics (e.g., messages and images), returning structured data over HTTP. It follows clean software architecture principles using the SOLID pattern and includes ROS abstraction, Pydantic validation, and extensible endpoint design.

---

## 🚀 Features

- ✅ Dynamic subscription to ROS 2 topics (auto message type detection)
- ✅ Supports `sensor_msgs/Image`, `std_msgs/String`, and all other ROS 2 messages
- ✅ Returns raw images as `image/png` via HTTP
- ✅ Returns all other message types as JSON
- ✅ Extensible architecture using ROS interface handlers
- ✅ `/get_data`, `/list_topics`, and `/list_nodes` endpoints
- ✅ Modular router structure with API versioning support

---

## 📦 Requirements

- ROS 2 (Humble or newer)
- Python 3.8+

### ROS dependencies (package.xml)

> ✅ All Python dependencies are declared in `package.xml` and can be installed with:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## 🔧 Installation

```bash
# Clone the project into your ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/mschweig/ros2_web_interface.git ros2_web_interface

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
cd ~/ros2_ws
colcon build --packages-select ros2_web_interface
source install/setup.bash
```

---

## 🚀 Usage

### Start the Web Server

```bash
ros2 run ros2_web_interface ros2_web_interface
```

It will start a FastAPI server at:
```
http://localhost:8000
```

---

## 📘 Example Endpoints

### 1. List all available topics
```http
GET /list_topics
```

### 2. List all active ROS nodes
```http
GET /list_nodes
```

### 3. Get latest data from a ROS 2 topic
```http
GET /get_data?topic=/your_topic_name
```
- If the topic is an image (`sensor_msgs/Image`), the server returns a raw `image/png`.
- For all other types, the server auto-detects the message type and returns structured JSON like:

```json
{
  "topic": "/some_topic",
  "type": "message",
  "message_type": "YourMsgType",
  "data": {
    "field1": 42,
    "field2": "abc"
  }
}
```

---

## 🧪 Testing

This project includes a test publisher and a `pytest` suite:

```bash
# Run test publishers (defined in params.yaml)
ros2 run ros2_web_interface test_publishers --ros-args --params-file src/ros2_web_interface/test/params.yaml

# Run integration tests
pytest src/ros2_web_interface/test
```

---

## 🏗 Architecture Overview

```
main.py             - Entry point, spins up ROS and FastAPI
models.py           - Pydantic request/response models
ros/
  ├─ base.py        - Abstract handler interface (ROSInterface)
  ├─ topic.py       - TopicHandler for dynamic topic data
  ├─ system.py      - SystemHandler for meta operations
  └─ factory.py     - Handler factory dispatcher
api/
  ├─ registry.py    - Central route registration
  ├─ topic_routes.py- /get_data
  └─ system_routes.py - /list_topics, /list_nodes
```

---

## 📚 License

Apache License 2.0. Built for real-world ROS 2 applications.

---

## 🙋 Need Help or Want to Contribute?
Pull requests welcome — especially for adding support for actions, services, or publisher endpoints!

---

Made with ❤️ by robotics engineers for real-world robots.

