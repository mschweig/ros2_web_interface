# ROS 2 Web Interface

[![License](https://img.shields.io/github/license/mschweig/ros2_web_interface.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Python](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)

A modular web interface for interacting with ROS 2 topics, services, and actions using FastAPI.

---

## 🚀 Features

- ✅ Dynamic subscription to ROS 2 topics (auto message type detection)
- ✅ Supports all ROS 2 messages (e.g., `sensor_msgs/Image`, `std_msgs/String`)
- ✅ Returns images as `image/png`, other messages as JSON
- ✅ Execute ROS 2 **services** via `/call_service`
- ✅ List available **actions** via `/list_actions`
- ✅ Execute **long-running actions** with:
  - `/call_action` → returns `goal_id`
  - `/action_result` → poll for status/result
- ✅ Persistent ROS node with background spinning
- ✅ Modular, extensible architecture with ROS interface handlers
- ✅ Versioned FastAPI routing for easy expansion

---

## 📦 Requirements

- ROS 2 (Humble or newer)
- Python 3.8+

> Install ROS dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## 🔧 Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/mschweig/ros2_web_interface.git ros2_web_interface
rosdep install --from-paths src --ignore-src -r -y
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

> Available at: `http://localhost:8000`

---

## 📘 Key Endpoints

### Topics:
- **`GET /list_topics`** – List all topics
- **`GET /get_data?topic=/your_topic`** – Fetch latest message/image

### Services:
- **`GET /list_services`** – List available services
- **`POST /call_service?topic=/robot/undock`** – Call service, body = `{}` or `{dock_id: 520}`

### Actions:
- **`GET /list_actions`** – List available actions
- **`POST /call_action?topic=/robot/dock`** – Send action goal:
  ```json
  { "dock_id": 520 }
  ```
  Response:
  ```json
  { "goal_id": "abc-123", "accepted": true }
  ```
- **`GET /action_result?goal_id=abc-123`** – Check status:
  ```json
  { "status": "pending" }
  ```
  or
  ```json
  { "status": "done", "result": { "success": true } }
  ```

---

## 🧪 Testing

```bash
ros2 run ros2_web_interface test_publishers --ros-args --params-file src/ros2_web_interface/test/params.yaml
pytest src/ros2_web_interface/test/
```

---

## 🏗 Architecture Overview

```
main.py             - ROS + FastAPI startup
models.py           - Pydantic models for validation & docs
ros/
  ├─ base.py        - Abstract handler (ROSInterface)
  ├─ topic.py       - Dynamic topic data
  ├─ service.py     - Call ROS services
  ├─ action.py      - Async action support
  ├─ system.py      - Topic/node/service listing
  └─ factory.py     - Handler factory
api/
  ├─ registry.py    - Route registration
  ├─ topic_routes.py
  ├─ service_routes.py
  └─ action_routes.py
```

---

## 📚 License

Apache License 2.0. Built for real-world ROS 2 applications.

---

## 🤛 Contribute?

Pull requests welcome — let's build robust web tools for ROS 2 robots together! 🤖❤️
