from __future__ import annotations

from dataclasses import asdict, dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class DetectorConfig:
    roi_top: float = 0.42
    roi_bottom: float = 1.0
    saturation_min: int = 90
    value_min: int = 55
    minimum_area_px: float = 180.0
    minimum_aspect_ratio: float = 2.0
    maximum_aspect_ratio: float = 12.0
    minimum_fill_ratio: float = 0.42


@dataclass(frozen=True)
class Detection:
    center_px: tuple[float, float]
    center_normalized: tuple[float, float]
    corners_px: tuple[tuple[float, float], ...]
    long_axis_angle_deg: float
    length_px: float
    width_px: float
    area_px: float
    aspect_ratio: float
    fill_ratio: float
    red_fraction: float
    confidence: float

    def to_dict(self) -> dict:
        value = asdict(self)
        value["center_px"] = list(self.center_px)
        value["center_normalized"] = list(self.center_normalized)
        value["corners_px"] = [list(point) for point in self.corners_px]
        return value


def _red_mask(bgr: np.ndarray, config: DetectorConfig) -> np.ndarray:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lower_red = cv2.inRange(
        hsv,
        np.array((0, config.saturation_min, config.value_min), np.uint8),
        np.array((12, 255, 255), np.uint8),
    )
    upper_red = cv2.inRange(
        hsv,
        np.array((168, config.saturation_min, config.value_min), np.uint8),
        np.array((179, 255, 255), np.uint8),
    )
    mask = cv2.bitwise_or(lower_red, upper_red)
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    )
    # Printed marks split the red label into several islands in the live ZED
    # image.  Close gaps along either possible strip direction before applying
    # the aspect-ratio test; nearby labels are much farther apart than 31 px.
    vertical = cv2.morphologyEx(
        mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 31))
    )
    horizontal = cv2.morphologyEx(
        mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (31, 5))
    )
    return cv2.bitwise_or(vertical, horizontal)


def detect_red_strips(
    bgr: np.ndarray, config: DetectorConfig = DetectorConfig()
) -> list[Detection]:
    if not isinstance(bgr, np.ndarray) or bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError("bgr must be an HxWx3 image")
    height, width = bgr.shape[:2]
    top = int(round(np.clip(config.roi_top, 0.0, 1.0) * height))
    bottom = int(round(np.clip(config.roi_bottom, 0.0, 1.0) * height))
    if bottom <= top:
        raise ValueError("ROI bottom must be below ROI top")

    mask = _red_mask(bgr, config)
    mask[:top] = 0
    mask[bottom:] = 0
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections: list[Detection] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < config.minimum_area_px:
            continue
        (cx, cy), (side_a, side_b), angle = cv2.minAreaRect(contour)
        length, strip_width = sorted((float(side_a), float(side_b)), reverse=True)
        if strip_width <= 1.0:
            continue
        aspect = length / strip_width
        rectangle_area = length * strip_width
        fill = area / max(rectangle_area, 1.0)
        if not config.minimum_aspect_ratio <= aspect <= config.maximum_aspect_ratio:
            continue
        if fill < config.minimum_fill_ratio:
            continue

        corners = cv2.boxPoints(((cx, cy), (side_a, side_b), angle))
        polygon = np.zeros(mask.shape, np.uint8)
        cv2.fillConvexPoly(polygon, np.rint(corners).astype(np.int32), 255)
        pixels = polygon > 0
        red_fraction = float(np.mean(mask[pixels] > 0)) if np.any(pixels) else 0.0
        long_angle = float(angle if side_a >= side_b else angle + 90.0)
        while long_angle >= 90.0:
            long_angle -= 180.0
        while long_angle < -90.0:
            long_angle += 180.0
        aspect_score = min(1.0, max(0.0, (aspect - config.minimum_aspect_ratio) / 2.5))
        confidence = float(np.clip(0.45 * red_fraction + 0.35 * fill + 0.20 * aspect_score, 0, 1))
        detections.append(
            Detection(
                center_px=(float(cx), float(cy)),
                center_normalized=(float(cx / width), float(cy / height)),
                corners_px=tuple((float(x), float(y)) for x, y in corners),
                long_axis_angle_deg=long_angle,
                length_px=length,
                width_px=strip_width,
                area_px=area,
                aspect_ratio=aspect,
                fill_ratio=fill,
                red_fraction=red_fraction,
                confidence=confidence,
            )
        )
    return sorted(detections, key=lambda item: (item.confidence, item.area_px), reverse=True)


def annotate(bgr: np.ndarray, detections: list[Detection]) -> np.ndarray:
    output = bgr.copy()
    for index, detection in enumerate(detections):
        corners = np.rint(np.asarray(detection.corners_px)).astype(np.int32)
        color = (0, 255, 255) if index == 0 else (0, 165, 255)
        cv2.polylines(output, [corners], True, color, 2, cv2.LINE_AA)
        center = tuple(np.rint(detection.center_px).astype(int))
        cv2.drawMarker(output, center, color, cv2.MARKER_CROSS, 18, 2)
        label = f"red strip {detection.confidence:.2f} {detection.long_axis_angle_deg:.1f}deg"
        cv2.putText(output, label, (center[0] + 8, center[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return output
