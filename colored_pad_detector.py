"""Detect the four colored strips used by the thermal-pad placement stage."""

from __future__ import annotations

from dataclasses import asdict, dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class ColoredPad:
    color: str
    center_px: tuple[float, float]
    bbox_xywh: tuple[int, int, int, int]
    area_px: float
    fill_ratio: float
    mean_saturation: float
    confidence: float

    def to_dict(self) -> dict:
        value = asdict(self)
        value["center_px"] = list(self.center_px)
        value["bbox_xywh"] = list(self.bbox_xywh)
        return value


def _mask(hsv: np.ndarray, color: str) -> np.ndarray:
    if color == "red":
        low = cv2.inRange(hsv, (0, 75, 45), (16, 255, 255))
        high = cv2.inRange(hsv, (165, 75, 45), (179, 255, 255))
        mask = cv2.bitwise_or(low, high)
    elif color == "green":
        mask = cv2.inRange(hsv, (35, 55, 35), (95, 255, 255))
    else:
        raise ValueError(f"unsupported color: {color}")
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    )
    return cv2.morphologyEx(
        mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (9, 17))
    )


def detect_colored_pads(
    bgr: np.ndarray,
    *,
    roi=(0.0, 0.0, 1.0, 1.0),
    minimum_area_px: float = 100.0,
    maximum_area_fraction: float = 0.15,
) -> list[ColoredPad]:
    if not isinstance(bgr, np.ndarray) or bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError("bgr must be an HxWx3 image")
    height, width = bgr.shape[:2]
    left, top, right, bottom = roi
    x0, y0 = int(left * width), int(top * height)
    x1, y1 = int(right * width), int(bottom * height)
    if not (0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height):
        raise ValueError("invalid normalized ROI")
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    detections: list[ColoredPad] = []
    for color in ("red", "green"):
        mask = _mask(hsv, color)
        roi_mask = np.zeros_like(mask)
        roi_mask[y0:y1, x0:x1] = mask[y0:y1, x0:x1]
        contours, _ = cv2.findContours(
            roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if not minimum_area_px <= area <= maximum_area_fraction * height * width:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if min(w, h) < 6:
                continue
            rect = cv2.minAreaRect(contour)
            side_a, side_b = map(float, rect[1])
            rectangle_area = max(side_a * side_b, 1.0)
            fill = area / rectangle_area
            if fill < 0.20:
                continue
            pixels = np.zeros(mask.shape, np.uint8)
            cv2.drawContours(pixels, [contour], -1, 255, -1)
            selected = pixels > 0
            mean_saturation = float(np.mean(hsv[..., 1][selected]))
            # White robot links and floor shadows can fall inside the broad
            # green hue interval, but their saturation stays low.  The four
            # printed task pads remain strongly saturated in the deployed ZED
            # stream, including the partially arm-occluded red pad.
            if mean_saturation < 100.0:
                continue
            confidence = float(np.clip(
                0.50 * fill + 0.35 * mean_saturation / 255.0
                + 0.15 * min(1.0, area / 1200.0),
                0.0,
                1.0,
            ))
            detections.append(ColoredPad(
                color=color,
                center_px=(x + w / 2.0, y + h / 2.0),
                bbox_xywh=(x, y, w, h),
                area_px=area,
                fill_ratio=fill,
                mean_saturation=mean_saturation,
                confidence=confidence,
            ))
    # Black print can fragment a strip, and hue uncertainty can create red and
    # green proposals at the same location. Keep one proposal per physical pad;
    # a red proposal wins a close tie because there is exactly one red target.
    accepted: list[ColoredPad] = []
    for candidate in sorted(
        detections,
        key=lambda item: (item.color == "red", item.confidence, item.area_px),
        reverse=True,
    ):
        if any(np.linalg.norm(
            np.asarray(candidate.center_px) - np.asarray(other.center_px)
        ) < 40.0 for other in accepted):
            continue
        accepted.append(candidate)
    return sorted(accepted, key=lambda item: item.center_px[0])


def map_layout_to_distances(
    detections: list[ColoredPad], distances_cm: list[float]
) -> dict:
    if len(detections) != len(distances_cm):
        raise ValueError(
            f"expected {len(distances_cm)} pads, detected {len(detections)}"
        )
    red = [item for item in detections if item.color == "red"]
    if len(red) != 1:
        raise ValueError(f"expected exactly one red pad, detected {len(red)}")
    ordered = sorted(detections, key=lambda item: item.center_px[0])
    distances = np.asarray(distances_cm, dtype=float)
    pixels = np.asarray([item.center_px[0] for item in ordered], dtype=float)
    slope, intercept = np.polyfit(pixels, distances, 1)
    predicted = slope * pixels + intercept
    pairs = [
        {
            "color": item.color,
            "center_px": list(item.center_px),
            "distance_from_black_base_cm": float(distance),
        }
        for item, distance in zip(ordered, distances)
    ]
    red_pair = next(pair for pair in pairs if pair["color"] == "red")
    return {
        "pairs": pairs,
        "linear_distance_cm_from_main_x_px": {
            "slope_cm_per_px": float(slope),
            "intercept_cm": float(intercept),
            "rmse_cm": float(np.sqrt(np.mean((predicted - distances) ** 2))),
        },
        "red_station_distance_cm": red_pair["distance_from_black_base_cm"],
    }


def best_red_wrist_target(bgr: np.ndarray) -> ColoredPad | None:
    candidates = [
        item for item in detect_colored_pads(
            bgr,
            roi=(0.05, 0.08, 0.95, 0.92),
            minimum_area_px=80.0,
            maximum_area_fraction=0.08,
        )
        if item.color == "red"
    ]
    # The wrist can see people, chairs and skin while its optical axis is not
    # table-facing. Those muted red regions caused an unsafe false lateral
    # correction during the first live trial. A real printed pad is saturated
    # and strip-like; reject background red before computing any base motion.
    candidates = [
        item for item in candidates
        if item.mean_saturation >= 130.0
        and max(item.bbox_xywh[2], item.bbox_xywh[3])
        / max(1, min(item.bbox_xywh[2], item.bbox_xywh[3])) >= 1.5
        and item.fill_ratio >= 0.25
    ]
    return max(candidates, key=lambda item: item.confidence, default=None)


def annotate(bgr: np.ndarray, detections: list[ColoredPad]) -> np.ndarray:
    output = bgr.copy()
    for item in detections:
        x, y, w, h = item.bbox_xywh
        color = (0, 0, 255) if item.color == "red" else (0, 220, 0)
        cv2.rectangle(output, (x, y), (x + w, y + h), color, 2)
        center = tuple(np.rint(item.center_px).astype(int))
        cv2.drawMarker(output, center, color, cv2.MARKER_CROSS, 18, 2)
        cv2.putText(
            output, f"{item.color} {item.confidence:.2f}",
            (x, max(15, y - 7)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
            cv2.LINE_AA,
        )
    return output
