import cv2
import numpy as np

from colored_pad_detector import (
    best_red_wrist_target,
    detect_colored_pads,
    map_layout_to_distances,
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
