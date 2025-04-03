# src/ros2_web_interface/test/conftest.py

import pytest
import yaml
from pathlib import Path

@pytest.fixture(scope="session")
def test_config():
    config_path = Path(__file__).parent / "params.yaml"
    with open(config_path, "r") as f:
        full_config = yaml.safe_load(f)
    
    topic_config = yaml.safe_load(full_config["test_publisher_node"]["ros__parameters"]["topics"])
    return {
        "chatter_topic": topic_config["chatter"]["name"],
        "image_topic": topic_config["camera_image"]["name"]
    }
