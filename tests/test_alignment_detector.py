import cv2
import numpy as np

from alignment_detector import (
    Target,
    detect_main_hint,
    detect_occluded_grey_pad,
    detect_target,
    horizontal_decision,
    wrist_vertical_robot_decision,
)
from align_to_thermal_pad import main_table_target, observe


def test_detects_grey_pad_on_black_base_and_requests_right():
    image = np.full((480, 640, 3), 235, np.uint8)
    cv2.rectangle(image, (450, 80), (610, 230), (20, 20, 20), -1)
    cv2.rectangle(image, (485, 145), (570, 178), (130, 130, 130), -1)
    target = detect_target(image)
    assert target is not None
    assert horizontal_decision(target, 640) == "move_right"


def test_rejects_plain_black_object_without_grey_pad():
    image = np.full((480, 640, 3), 235, np.uint8)
    cv2.rectangle(image, (220, 100), (390, 260), (15, 15, 15), -1)
    assert detect_target(image) is None


def test_main_hint_tolerates_occluded_black_base():
    image = np.full((720, 1280, 3), 230, np.uint8)
    cv2.rectangle(image, (570, 340), (690, 490), (20, 20, 20), -1)
    cv2.rectangle(image, (596, 365), (617, 444), (90, 90, 90), -1)
    cv2.rectangle(image, (620, 300), (680, 470), (20, 20, 20), -1)
    hint = detect_main_hint(image)
    assert hint is not None
    assert abs(hint.center[0] - 606.5) < 5


def test_rejects_dark_gripper_shape_touching_bottom_edge():
    image = np.full((480, 640, 3), 230, np.uint8)
    cv2.rectangle(image, (0, 300), (180, 479), (15, 15, 15), -1)
    cv2.rectangle(image, (60, 350), (150, 390), (120, 120, 120), -1)
    assert detect_target(image) is None


def test_wrist_vertical_axis_maps_up_to_robot_left():
    target = type("T", (), {"center": (500.0, 120.0)})()
    assert wrist_vertical_robot_decision(target, 480) == "move_left"
    target = type("T", (), {"center": (500.0, 360.0)})()
    assert wrist_vertical_robot_decision(target, 480) == "move_right"


def test_detects_pad_when_black_base_merges_with_gripper():
    image = np.full((480, 640, 3), 170, np.uint8)
    cv2.rectangle(image, (460, 180), (639, 300), (15, 15, 15), -1)
    cv2.rectangle(image, (495, 245), (635, 280), (90, 90, 90), -1)
    cv2.rectangle(image, (610, 270), (639, 479), (15, 15, 15), -1)
    target = detect_occluded_grey_pad(image)
    assert target is not None
    assert 250 < target.center[1] < 275


def test_occluded_fallback_rejects_grey_strip_without_black_base():
    image = np.full((480, 640, 3), 170, np.uint8)
    cv2.rectangle(image, (440, 230), (610, 270), (90, 90, 90), -1)
    assert detect_occluded_grey_pad(image) is None


def test_main_target_rejects_candidate_above_table_band(monkeypatch):
    image = np.full((720, 1280, 3), 170, np.uint8)
    candidate = Target((300.0, 70.0), (220, 20, 160, 120), 12000.0, 0.8)
    monkeypatch.setattr("align_to_thermal_pad.detect_target", lambda _: candidate)
    assert main_table_target(image) is None


def test_main_center_is_search_handoff_not_not_visible(monkeypatch):
    image = np.full((720, 1280, 3), 170, np.uint8)
    target = Target((640.0, 400.0), (600, 360, 80, 80), 6400.0, 0.9)
    monkeypatch.setattr("align_to_thermal_pad.frame", lambda _: image)
    monkeypatch.setattr("align_to_thermal_pad.detect_target", lambda _: None)
    monkeypatch.setattr("align_to_thermal_pad.detect_occluded_grey_pad", lambda _: None)
    monkeypatch.setattr("align_to_thermal_pad.main_table_target", lambda _: target)
    state = observe()
    assert state["main_visible"] is True
    assert state["wrist_visible"] is False
    assert state["decision"] == "search_from_main_center"
