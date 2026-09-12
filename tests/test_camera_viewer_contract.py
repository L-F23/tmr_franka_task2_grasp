from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "policy" / "camera_viewer.py").read_text(encoding="utf-8")


def test_main_camera_consumes_a_compressed_ros_topic_directly():
    assert 'DEFAULT_MAIN_TOPIC = "/tmr_task2/zed/image/compressed"' in SOURCE
    assert "TMR_MAIN_CAMERA_TOPIC" in SOURCE
    assert "CompressedImage" in SOURCE


def test_viewer_exposes_in_container_snapshots_for_policy_subprocesses():
    assert 'path in ("/main.jpg", "/left.jpg")' in SOURCE
    assert '("localhost", args.port)' in SOURCE


def test_only_required_left_wrist_camera_joins_the_humble_graph():
    assert '"left": "/wrist_camera_left/camera/color/image_rect_raw"' in SOURCE
    assert '"right"' not in SOURCE
    assert "qos_profile_sensor_data" in SOURCE


def test_zed_dds_domain_bridge_forwards_only_the_compressed_topic():
    bridge = (ROOT / "docker" / "zed_domain_bridge.yaml").read_text(
        encoding="utf-8"
    )
    assert "from_domain: 1" in bridge
    assert "to_domain: 0" in bridge
    assert "/head_camera/zed/rgb/color/rect/image/compressed:" in bridge
    assert "type: sensor_msgs/msg/CompressedImage" in bridge
    assert "remap: /tmr_task2/zed/image/compressed" in bridge
    assert bridge.count("type: sensor_msgs/msg/") == 1


def test_live_detector_uses_the_bundled_viewer_not_a_stale_testbed_ip():
    detector = (ROOT / "policy" / "detect_red_strip.py").read_text(encoding="utf-8")
    source = (ROOT / "policy" / "red_strip_detector" / "source.py").read_text(
        encoding="utf-8"
    )
    assert "http://localhost:18081/main.jpg" in detector
    assert "172.16." not in detector
    assert 'response.headers.get("X-TMR-Frame-Sequence")' in source
