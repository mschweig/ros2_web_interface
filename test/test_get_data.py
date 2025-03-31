import requests

def test_chatter_topic(test_config):
    topic_cfg = test_config["chatter"]
    topic_name = topic_cfg["name"]
    expected = topic_cfg["test_message"]

    res = requests.get("http://localhost:8000/get_data", params={"topic": topic_name})
    assert res.status_code == 200
    data = res.json()

    assert data["type"] == "message"
    assert expected in data["data"]["data"]
    print(f"[OK] Received expected message from {topic_name}: {data['data']['data']}")
