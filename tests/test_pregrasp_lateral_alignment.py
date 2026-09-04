import cv2

from pregrasp_lateral_alignment import (
    main_guidance,
    mapped_correction_m,
    mapped_wrist_state,
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


def test_measured_mapping_has_verified_wrist_y_direction():
    import json

    mapping = json.load(open("config/wrist_lateral_mapping.json"))
    assert mapped_correction_m(-20.0, mapping) > 0.0
    assert mapped_correction_m(20.0, mapping) < 0.0


def test_measured_mapping_reference_is_aligned():
    import json

    mapping = json.load(open("config/wrist_lateral_mapping.json"))
    image = cv2.imread(mapping["reference_image"])
    result = mapped_wrist_state(image, mapping, 6.0)
    assert result["decision"] == "aligned"
    assert result["target_center_y_error_px"] == 0.0


def test_visible_target_outside_calibrated_range_keeps_wrist_direction():
    import json
    import numpy as np

    mapping = json.load(open("config/wrist_lateral_mapping.json"))
    reference = cv2.imread(mapping["reference_image"])
    shifted = np.full_like(reference, 190)
    x, y, width, height = mapping["reference_bbox_xywh"]
    shifted[y + 80:y + 80 + height, x:x + width] = reference[y:y + height, x:x + width]
    result = mapped_wrist_state(shifted, mapping, 6.0)
    assert result["inside_calibrated_range"] is False
    assert result["decision"] == "move_left"
    assert result["mapping_use"] == "operator_verified_direction_with_step_clamp"
