# ROS 2 Web Interface

[![License](https://img.shields.io/github/license/mschweig/ros2_web_interface.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Python](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)

A high-performance, modular web interface for interacting with ROS 2 topics, services, and actions via FastAPI.

This project provides a robust and extensible framework for dynamic interaction with ROS 2 systems over HTTP, supporting quick data access, service execution, and action handling. Designed with clean software architecture principles, it integrates ROS 2 with modern web technologies to enhance robotics development workflows.

---

## 🚀 Key Features

- ✅ **Dynamic Topic Subscription**: Auto-detects ROS 2 message types.
- ✅ **Full Message Support**: Compatible with all ROS 2 message types, including `sensor_msgs/Image`.
- ✅ **Media-Aware Responses**: Serves images as `image/png`, other messages as structured JSON.
- ✅ **Action Handling**: Submit and monitor ROS 2 actions.
- ✅ **Extensible Architecture**: Modular handler system for easy feature expansion.
- ✅ **Persistent Node**: Background spinning ensures continuous ROS 2 interaction.
- ✅ **API Versioning**: Structured routes ready for scalable API evolution.
- ✅ **Interactive API Documentation**: Swagger UI available at `/docs`.

---

## 📦 Requirements

- ROS 2 (Humble or newer)
- Python 3.8 or higher

> Install all dependencies via:
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

## 🚀 Running the Web Server

```bash
ros2 run ros2_web_interface ros2_web_interface
```

> Access the API at: `http://localhost:8000`

---

## 🔍 Interactive API Documentation

This project provides interactive documentation via **Swagger UI**:

- Access at: `http://localhost:8000/docs`
- Visualize available endpoints.
- Test API calls directly from the browser.
- Auto-generated from Pydantic models for accuracy and clarity.

---

## 🧪 Testing Suite

```bash
ros2 run ros2_web_interface test_publishers --ros-args --params-file src/ros2_web_interface/test/params.yaml
pytest src/ros2_web_interface/test/
```

---

## 🏗️ Architecture Overview

```
main.py               - ROS node + FastAPI startup logic
models.py             - Pydantic schemas for validation and documentation
ros/
  ├─ base.py        - Abstract handler interface (ROSInterface)
  ├─ topic.py       - TopicHandler for dynamic topic data
  ├─ service.py     - ServiceHandler for ROS service calls
  ├─ action.py      - ActionHandler for action management
  ├─ system.py      - SystemHandler for metadata operations
  └─ factory.py     - Dynamic handler factory
api/
  ├─ registry.py    - Central route registry
  ├─ topic_routes.py
  ├─ service_routes.py
  └─ action_routes.py
```

---

## 📚 License

Apache License 2.0 — Optimized for professional ROS 2 deployments.

---

## 🤛 Contributing

Contributions are welcome! Help us enhance ROS 2 web integration by submitting issues, feature requests, or pull requests.

---

Crafted with ❤️ by robotics engineers for real-world ROS 2 applications.
