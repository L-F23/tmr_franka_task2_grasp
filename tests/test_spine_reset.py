import json

import pytest

from reset_spine_to_task_height import reset_spine


def write_config(path, target_m=0.6):
    path.write_text(json.dumps({
        "spine": {
            "target_position_m": target_m,
            "velocity_m_s": 0.05,
            "acceleration_m_s2": 0.1,
            "deceleration_m_s2": 0.1,
            "maximum_final_error_m": 0.003,
        }
    }), encoding="utf-8")


def test_spine_reset_uses_canonical_height_without_moving_arms(monkeypatch, tmp_path):
    config = tmp_path / "initial_pose.json"
    record = tmp_path / "latest_spine_reset.json"
    write_config(config)
    monkeypatch.setattr(
        "reset_spine_to_task_height.move_spine",
        lambda _config: {
            "start_position_m": 0.58,
            "target_position_m": 0.6,
            "measured_position_m": 0.6,
        },
    )

    report = reset_spine(config, record)

    assert report["status"] == "spine_task_height_restored"
    assert report["spine_commanded"] is True
    assert report["left_arm_commanded"] is False
    assert report["right_arm_commanded"] is False
    assert json.loads(record.read_text(encoding="utf-8"))["status"] == report["status"]


def test_spine_reset_rejects_noncanonical_target(tmp_path):
    config = tmp_path / "initial_pose.json"
    write_config(config, target_m=0.55)
    with pytest.raises(ValueError, match="canonical Spine target"):
        reset_spine(config, tmp_path / "record.json")
