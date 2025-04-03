import requests

def test_image_topic(test_config):
    url = f"http://localhost:8000/get_data?topic={test_config['image_topic']}"
    response = requests.get(url)
    assert response.status_code == 200
    assert response.headers["Content-Type"] == "image/png"
    assert len(response.content) > 1000
