from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "policy" / "camera_viewer.py").read_text(encoding="utf-8")


def test_main_camera_uses_base_local_http_export():
    assert "http://172.16.0.50:18082/tmr_zed_latest.jpg" in SOURCE
    assert "ProxyHandler({})" in SOURCE
    assert "CompressedImage" not in SOURCE


def test_wrist_cameras_remain_in_jazzy_ros_graph():
    assert '"left": "/wrist_camera_left/color/image_raw"' in SOURCE
    assert '"right": "/wrist_camera_right/color/image_raw"' in SOURCE
    assert "qos_profile_sensor_data" in SOURCE
