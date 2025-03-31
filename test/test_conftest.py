import pytest
import yaml
from pathlib import Path
import requests
import time

@pytest.fixture(scope="session")
def test_config():
    # Load params.yaml and extract topic mappings
    param_file = Path(__file__).parent / "params.yaml"
    with open(param_file, "r") as f:
        full_config = yaml.safe_load(f)

    topics_yaml = full_config["test_publisher_node"]["ros__parameters"]["topics"]
    topics = yaml.safe_load(topics_yaml)
    return topics

@pytest.fixture(scope="session", autouse=True)
def wait_for_api():
    print("[pytest] Waiting for FastAPI service...")
    for _ in range(30):
        try:
            res = requests.get("http://localhost:8000/list_topics")
            if res.status_code == 200:
                print("[pytest] API is up.")
                return
        except:
            pass
        time.sleep(1)
    pytest.fail("FastAPI server not available.")
