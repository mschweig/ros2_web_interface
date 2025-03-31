import requests
import base64

def test_image_topic(test_config):
    topic_cfg = test_config["camera_image"]
    topic_name = topic_cfg["name"]

    res = requests.get("http://localhost:8000/get_data", params={"topic": topic_name})
    assert res.status_code == 200

    data = res.json()
    assert data["type"] == "image"
    assert data["format"] == "png"
    assert "base64" in data

    img_data = base64.b64decode(data["base64"])
    with open("test_output_image.png", "wb") as f:
        f.write(img_data)
    print(f"[OK] Image from {topic_name} saved to test_output_image.png")
