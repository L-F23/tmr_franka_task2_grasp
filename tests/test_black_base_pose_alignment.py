import cv2

from black_base_pose_alignment import match_base_structure
from thermal_pad_ik import ROOT


def test_three_template_consensus_tracks_known_lateral_sample():
    reference = cv2.imread(str(
        ROOT / "captures/wrist_lateral_mapping_20260904/zero_left.jpg"
    ))
    shifted = cv2.imread(str(
        ROOT / "captures/wrist_lateral_mapping_20260904/sample_06.jpg"
    ))
    result = match_base_structure(shifted, reference)
    assert result["minimum_confidence"] > 0.82
    assert 0.94 <= result["median_scale"] <= 1.04
    assert result["median_center_y_error_px"] > 45.0
    assert result["scale_spread"] <= 0.08
