import cv2
import numpy as np

from red_strip_detector.detector import DetectorConfig, detect_red_strips


def canvas() -> np.ndarray:
    return np.full((720, 1280, 3), (205, 205, 205), np.uint8)


def test_detects_vertical_red_strip_and_reports_center() -> None:
    image = canvas()
    box = cv2.boxPoints(((540, 515), (22, 84), 1.5)).astype(np.int32)
    cv2.fillConvexPoly(image, box, (0, 0, 220))
    found = detect_red_strips(image)
    assert len(found) == 1
    assert abs(found[0].center_px[0] - 540) < 2
    assert abs(found[0].center_px[1] - 515) < 2
    assert found[0].aspect_ratio > 3.0
    assert abs(abs(found[0].long_axis_angle_deg) - 90.0) < 4.0


def test_joins_printed_red_strip_fragments() -> None:
    image = canvas()
    for top, bottom in ((455, 480), (491, 516), (527, 552)):
        cv2.rectangle(image, (530, top), (548, bottom), (0, 0, 220), -1)
    found = detect_red_strips(image)
    assert len(found) == 1
    assert found[0].length_px > 90
    assert found[0].aspect_ratio > 4


def test_ignores_red_objects_above_table_roi() -> None:
    image = canvas()
    cv2.rectangle(image, (200, 80), (225, 190), (0, 0, 255), -1)
    assert detect_red_strips(image) == []


def test_ignores_square_red_patch() -> None:
    image = canvas()
    cv2.rectangle(image, (500, 450), (550, 500), (0, 0, 255), -1)
    assert detect_red_strips(image) == []


def test_detects_both_red_hue_wrap_ranges() -> None:
    hsv = np.zeros((720, 1280, 3), np.uint8)
    hsv[:] = (0, 0, 205)
    cv2.rectangle(hsv, (400, 460), (420, 550), (2, 240, 230), -1)
    cv2.rectangle(hsv, (700, 460), (720, 550), (177, 240, 230), -1)
    image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    assert len(detect_red_strips(image)) == 2


def test_rejects_invalid_roi() -> None:
    image = canvas()
    try:
        detect_red_strips(image, DetectorConfig(roi_top=0.9, roi_bottom=0.2))
    except ValueError as exc:
        assert "ROI" in str(exc)
    else:
        raise AssertionError("invalid ROI was accepted")
