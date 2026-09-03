import cv2
import numpy as np

from alignment_detector import detect_main_hint, detect_target, horizontal_decision


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
