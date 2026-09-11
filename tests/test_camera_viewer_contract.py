from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "policy" / "camera_viewer.py").read_text(encoding="utf-8")


def test_main_camera_prefers_evaluator_ros_topic_with_http_fallback():
    assert 'DEFAULT_MAIN_TOPIC = "/head_camera/zed/rgb/color/rect/image/compressed"' in SOURCE
    assert "TMR_MAIN_CAMERA_TOPIC" in SOURCE
    assert "CompressedImage" in SOURCE
    assert 'DEFAULT_MAIN_URL = ""' in SOURCE
    assert "ProxyHandler({})" in SOURCE


def test_only_required_left_wrist_camera_joins_the_humble_graph():
    assert '"left": "/wrist_camera_left/color/image_raw"' in SOURCE
    assert '"right"' not in SOURCE
    assert "qos_profile_sensor_data" in SOURCE
