import math
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from stage0_wall_dock_and_find_stand import (
    detect_black_stand,
    execute,
    load_config,
    next_right_search_step,
)
from stage0_wall_docking_base import (
    fit_rear_wall,
    wall_control_targets,
    wall_distance_intent,
)


def synthetic_wall(clearance_m=0.22, angle_deg=0.0):
    generator = np.random.default_rng(7)
    y = np.linspace(-1.0, 1.0, 180)
    slope = math.tan(math.radians(angle_deg))
    x = -(0.40 + clearance_m) + slope * y
    x += generator.normal(0.0, 0.004, size=x.shape)
    wall = list(zip(x.tolist(), y.tolist()))
    clutter = [(-0.8, -0.15), (-1.1, 0.25), (-0.5, 0.05)]
    return wall + clutter


def test_rear_wall_fit_uses_body_clearance_and_recovers_angle():
    report = fit_rear_wall(synthetic_wall(clearance_m=0.22, angle_deg=4.0))
    assert report["rear_body_clearance_m"] == pytest.approx(0.22, abs=0.008)
    assert report["wall_angle_error_deg"] == pytest.approx(4.0, abs=0.5)
    assert report["lateral_support_m"] > 1.5


def test_parallel_wall_has_matching_left_and_right_clearance():
    report = fit_rear_wall(synthetic_wall(clearance_m=0.22, angle_deg=0.0))
    slope = report["slope_x_per_y"]
    intercept = report["intercept_x_m"]
    left = -(slope * 0.5 + intercept) - 0.40
    right = -(slope * -0.5 + intercept) - 0.40
    assert left - right == pytest.approx(0.0, abs=0.008)


def test_wall_control_moves_backward_when_wall_is_too_far_and_corrects_yaw():
    vx, wz = wall_control_targets(0.35, 5.0, 0.22, 0.025, 0.08)
    assert vx < 0.0
    assert wz < 0.0
    vx, wz = wall_control_targets(0.15, -5.0, 0.22, 0.025, 0.08)
    assert vx > 0.0
    assert wz > 0.0


def test_wall_distance_intent_reports_without_commanding_translation():
    backward = wall_distance_intent(0.35, 0.22)
    assert backward["direction"] == "backward"
    assert backward["distance_m"] == pytest.approx(0.13)
    assert backward["translation_commanded"] is False

    forward = wall_distance_intent(0.15, 0.22)
    assert forward["direction"] == "forward"
    assert forward["distance_m"] == pytest.approx(0.07)
    assert forward["translation_commanded"] is False

    hold = wall_distance_intent(0.225, 0.22)
    assert hold["direction"] == "hold"
    assert hold["distance_m"] == 0.0


def test_black_stand_detector_requires_dark_object_on_bright_table():
    image = np.full((720, 1280, 3), 215, dtype=np.uint8)
    cv2.rectangle(image, (800, 360), (1030, 500), (20, 20, 20), -1)
    detection = detect_black_stand(image)
    assert detection is not None
    assert detection["detector"] == "black_stand_only"
    assert detection["center_px"] == pytest.approx([915.5, 430.5], abs=2.0)
    assert detect_black_stand(np.full_like(image, 215)) is None


def test_black_stand_detector_rejects_recorded_robot_foreground():
    sample = cv2.imread("captures/20260903_222646_zed.jpg")
    assert sample is not None
    assert detect_black_stand(sample) is None


def test_right_search_never_commands_beyond_3_2m():
    requested = 0.0
    commands = []
    while True:
        step = next_right_search_step(requested, 3.2, 0.08)
        if step is None:
            break
        commands.append(step)
        requested += step
    assert requested == pytest.approx(3.2)
    assert len(commands) == 40
    assert next_right_search_step(requested, 3.2, 0.08) is None


def test_execute_stops_for_distance_confirmation_before_right_search(
    monkeypatch, tmp_path,
):
    calls = []

    def fake_base_controller(arguments, _timeout_s):
        calls.append(arguments)
        if arguments[arguments.index("--mode") + 1] == "rotate":
            return {"status": "success", "phase": "rotate_ccw"}
        assert "--orientation-only" in arguments
        return {
            "status": "awaiting_distance_confirmation",
            "phase": "rear_wall_heading_aligned",
            "distance_intent": {
                "direction": "backward",
                "distance_m": 0.13,
                "translation_commanded": False,
            },
        }

    def forbidden_right_motion(*_args, **_kwargs):
        raise AssertionError("right search must wait for operator confirmation")

    monkeypatch.setattr(
        "stage0_wall_dock_and_find_stand.run_base_controller",
        fake_base_controller,
    )
    monkeypatch.setattr(
        "stage0_wall_dock_and_find_stand.guarded_move_right",
        forbidden_right_motion,
    )
    config = load_config(Path("config/wall_dock_search.json"))
    record = tmp_path / "record.json"
    assert execute(config, record) == 0
    saved = json.loads(record.read_text(encoding="utf-8"))
    assert saved["status"] == "awaiting_wall_distance_confirmation"
    assert saved["distance_intent"]["direction"] == "backward"
    assert saved["right_search_deferred"] is True
    assert len(calls) == 2
