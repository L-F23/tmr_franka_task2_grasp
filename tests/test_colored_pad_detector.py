import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from colored_pad_detector import (
    ColoredPad,
    best_red_wrist_target,
    detect_colored_pads,
    estimate_red_station_from_reference,
    fit_center_distance_model,
    map_layout_to_distances,
    predict_distance_from_center,
)
from base_motion import split_lateral_move


def synthetic_layout():
    image = np.full((400, 800, 3), 235, np.uint8)
    colors = [(0, 0, 220), (0, 180, 0), (0, 180, 0), (0, 180, 0)]
    for x, color in zip((250, 380, 510, 640), colors):
        cv2.rectangle(image, (x - 12, 220), (x + 12, 310), color, -1)
        cv2.rectangle(image, (x - 6, 245), (x + 6, 255), (10, 10, 10), -1)
    return image


def test_detects_one_red_and_three_green_pads():
    detections = detect_colored_pads(
        synthetic_layout(), roi=(0.2, 0.45, 0.95, 0.85)
    )
    assert [item.color for item in detections] == ["red", "green", "green", "green"]


def test_maps_sorted_visual_centers_to_known_distances():
    detections = detect_colored_pads(
        synthetic_layout(), roi=(0.2, 0.45, 0.95, 0.85)
    )
    result = map_layout_to_distances(detections, [19.5, 32.9, 44.6, 58.0])
    assert result["red_station_distance_cm"] == 19.5
    assert result["linear_distance_cm_from_main_x_px"]["rmse_cm"] < 0.5


def test_saved_centers_form_low_residual_quadratic_distance_model():
    calibration = json.loads(Path(
        "config/red_pad_center_calibration.json"
    ).read_text(encoding="utf-8"))
    model = fit_center_distance_model(calibration["anchors"], degree=2)
    assert model["rmse_cm"] < 0.10
    for anchor in calibration["anchors"]:
        prediction = predict_distance_from_center(
            anchor["center_px"], model,
            maximum_cross_track_px=45.0,
            maximum_extrapolation_px=35.0,
        )
        assert prediction["distance_from_black_base_cm"] == pytest.approx(
            anchor["distance_from_black_base_cm"], abs=0.15
        )


def test_red_center_is_recognized_independently_and_evaluated_by_fit():
    calibration = json.loads(Path(
        "config/red_pad_center_calibration.json"
    ).read_text(encoding="utf-8"))
    red = ColoredPad(
        color="red",
        center_px=(1046.0, 429.0),
        bbox_xywh=(997, 388, 69, 76),
        area_px=1697.0,
        fill_ratio=0.8,
        mean_saturation=170.0,
        confidence=0.9,
    )
    result = estimate_red_station_from_reference(
        [red], calibration, image_size_px=(1280, 720)
    )
    assert result["detected_red_center_px"] == [1046.0, 429.0]
    assert result["distance_from_black_base_cm"] == pytest.approx(49.3, abs=0.15)


def test_long_coarse_move_is_split_into_guarded_steps():
    assert np.allclose(split_lateral_move(0.195), [0.08, 0.08, 0.035])


def test_wrist_target_rejects_muted_background_red():
    image = np.full((300, 400, 3), 220, np.uint8)
    cv2.rectangle(image, (150, 80), (190, 210), (105, 85, 155), -1)
    assert best_red_wrist_target(image) is None


def test_wrist_target_accepts_saturated_red_strip():
    image = np.full((300, 400, 3), 230, np.uint8)
    cv2.rectangle(image, (180, 90), (205, 230), (0, 0, 220), -1)
    assert best_red_wrist_target(image) is not None
