import cv2

from pregrasp_lateral_alignment import (
    main_guidance,
    wrist_decision,
)
from alignment_detector import detect_occluded_grey_pad, detect_target


CONFIG = {
    "wrist_reference_image": "captures/grasp_lateral_reference_20260904/aligned_left.jpg",
    "wrist_template_bbox_xywh": [300, 220, 145, 195],
}


def test_operator_verified_wrist_direction_mapping():
    assert wrist_decision(170, 220, 20) == "move_right"
    assert wrist_decision(270, 220, 20) == "move_left"
    assert wrist_decision(225, 220, 20) == "aligned"


def test_legacy_reference_is_rejected_as_a_structured_target():
    image = cv2.imread(CONFIG["wrist_reference_image"])
    assert detect_target(image) is None
    assert detect_occluded_grey_pad(image) is None


def test_main_reference_guides_from_red_pad_displacement():
    image = cv2.imread("captures/grasp_lateral_reference_20260904/aligned_main.jpg")
    # Shift the reference value left of the measured red center: the current
    # red pad is to the right, so the base must move right.
    result = main_guidance(image, 700.0, 20.0)
    assert result["decision"] == "move_right"
