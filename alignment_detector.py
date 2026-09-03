"""Detect the black base and its grey thermal pad in table-camera images."""

from __future__ import annotations

from dataclasses import dataclass
import cv2
import numpy as np


@dataclass(frozen=True)
class Target:
    center: tuple[float, float]
    base_box: tuple[int, int, int, int]
    area: float
    confidence: float


def detect_target(image: np.ndarray) -> Target | None:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    value, saturation = hsv[:, :, 2], hsv[:, :, 1]
    dark = np.uint8((value < 85) & (saturation < 150)) * 255
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    height, width = image.shape[:2]
    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        if x <= 3 or y + h >= int(0.82 * height):
            continue
        if area < 0.008 * width * height or w < 45 or h < 35:
            continue
        rectangularity = area / max(1.0, w * h)
        if rectangularity < 0.45 or max(w / h, h / w) > 2.4:
            continue
        # The low-saturation mid-tone insert must overlap the black base box.
        roi_v = value[y:y+h, x:x+w]
        roi_s = saturation[y:y+h, x:x+w]
        grey = np.uint8((roi_v >= 70) & (roi_v <= 205) & (roi_s < 75)) * 255
        grey = cv2.morphologyEx(grey, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        grey_contours, _ = cv2.findContours(grey, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        grey_contours = [c for c in grey_contours if cv2.contourArea(c) >= 90]
        if not grey_contours:
            continue
        pad = max(grey_contours, key=cv2.contourArea)
        moments = cv2.moments(pad)
        if moments["m00"] <= 0:
            continue
        cx = x + moments["m10"] / moments["m00"]
        cy = y + moments["m01"] / moments["m00"]
        confidence = min(1.0, 0.55 * rectangularity + 0.45 * min(1.0, cv2.contourArea(pad) / 900))
        candidates.append(Target((cx, cy), (x, y, w, h), area, confidence))
    return max(candidates, key=lambda item: item.confidence) if candidates else None


def horizontal_decision(target: Target | None, width: int, deadband_px: int = 45) -> str:
    if target is None:
        return "not_visible"
    error = target.center[0] - width / 2
    if abs(error) <= deadband_px:
        return "centered"
    return "move_right" if error > 0 else "move_left"


def wrist_vertical_robot_decision(target: Target | None, height: int,
                                  deadband_px: int = 35) -> str:
    """Initial-pose calibration: image up/down maps to robot left/right."""
    if target is None:
        return "not_visible"
    error = target.center[1] - height / 2
    if abs(error) <= deadband_px:
        return "centered"
    return "move_left" if error < 0 else "move_right"


def detect_main_hint(image: np.ndarray) -> Target | None:
    """Find the exposed grey insert when the arm occludes the main-view base."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    value, saturation = hsv[:, :, 2], hsv[:, :, 1]
    height, width = image.shape[:2]
    mask = np.uint8((value >= 55) & (value <= 130) & (saturation < 70)) * 255
    allowed = np.zeros_like(mask)
    allowed[int(0.34 * height):int(0.88 * height), int(0.23 * width):int(0.90 * width)] = 255
    mask = cv2.morphologyEx(mask & allowed, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    choices = []
    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        elongation = max(w / max(h, 1), h / max(w, 1))
        if area < 350 or not 2.2 <= elongation <= 6.5:
            continue
        # Require dark pixels immediately around the insert, representing its base.
        margin = 35
        y0, y1 = max(0, y-margin), min(height, y+h+margin)
        x0, x1 = max(0, x-margin), min(width, x+w+margin)
        dark_fraction = float(np.mean(value[y0:y1, x0:x1] < 55))
        if dark_fraction < 0.12:
            continue
        moments = cv2.moments(contour)
        center = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])
        choices.append(Target(center, (x, y, w, h), area, min(1.0, 0.6 + dark_fraction)))
    return max(choices, key=lambda item: item.confidence) if choices else None
