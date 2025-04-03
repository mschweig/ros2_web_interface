import requests

def test_chatter_topic(test_config):
    url = f"http://localhost:8000/get_data?topic={test_config['chatter_topic']}"
    response = requests.get(url)
    assert response.status_code == 200
    data = response.json()
    assert data["topic"] == test_config["chatter_topic"]
    assert data["message_type"] == "String"
    assert "data" in data["data"]
