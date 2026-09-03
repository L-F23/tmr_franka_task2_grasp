"""Pure vision and geometry helpers for the left-wrist thermal-pad pick."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class PadObservation:
    center_uv: tuple[float, float]
    grasp_uv: tuple[float, float]
    long_axis_uv: tuple[float, float]
    size_px: tuple[float, float]
    area_px: float
    mask: np.ndarray


def detect_pad_end(
    bgr: np.ndarray,
    near_end_direction: tuple[float, float] = (1.0, 0.0),
    endpoint_inset_fraction: float = 0.14,
) -> PadObservation | None:
    """Locate the grey pad and select its calibrated robot-near end.

    In the recorded initial left-wrist pose the end nearest the robot points
    toward positive image X.  The direction remains configurable because a
    remounted camera changes that relationship.
    """
    if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3:
        return None
    height, width = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    saturation, value = hsv[:, :, 1], hsv[:, :, 2]
    mask = np.uint8((value >= 42) & (value <= 175) & (saturation <= 82)) * 255
    allowed = np.zeros_like(mask)
    allowed[int(0.06 * height):int(0.92 * height), int(0.24 * width):] = 255
    mask &= allowed
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    choices: list[tuple[float, np.ndarray, tuple]] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < max(280.0, 0.0012 * width * height):
            continue
        rect = cv2.minAreaRect(contour)
        (_, _), (a, b), _ = rect
        long_side, short_side = max(a, b), min(a, b)
        if short_side < 8 or long_side / max(short_side, 1.0) < 2.2:
            continue
        if long_side > 0.58 * width or short_side > 0.24 * height:
            continue
        # Prefer an elongated, sizeable region close to the vertical center.
        cy = float(rect[0][1])
        score = area * min(long_side / max(short_side, 1.0), 6.0)
        score /= 1.0 + abs(cy - height / 2.0) / height
        choices.append((score, contour, rect))
    if not choices:
        return None

    _, contour, rect = max(choices, key=lambda item: item[0])
    center = np.asarray(rect[0], dtype=np.float64)
    box = cv2.boxPoints(rect).astype(np.float64)
    edges = np.roll(box, -1, axis=0) - box
    axis = edges[int(np.argmax(np.linalg.norm(edges, axis=1)))]
    axis /= np.linalg.norm(axis)
    direction = np.asarray(near_end_direction, dtype=np.float64)
    if np.linalg.norm(direction) < 1e-9:
        raise ValueError("near_end_direction must be non-zero")
    if float(np.dot(axis, direction)) < 0:
        axis = -axis
    long_side, short_side = sorted(map(float, rect[1]), reverse=True)
    reach = 0.5 * long_side * (1.0 - 2.0 * float(endpoint_inset_fraction))
    grasp = center + axis * reach

    selected = np.zeros_like(mask)
    cv2.drawContours(selected, [contour], -1, 255, thickness=-1)
    return PadObservation(
        center_uv=(float(center[0]), float(center[1])),
        grasp_uv=(float(grasp[0]), float(grasp[1])),
        long_axis_uv=(float(axis[0]), float(axis[1])),
        size_px=(long_side, short_side),
        area_px=float(cv2.contourArea(contour)),
        mask=selected,
    )


def register_depth_point(
    depth_m: np.ndarray,
    depth_intr: Intrinsics,
    color_intr: Intrinsics,
    rotation_depth_to_color: np.ndarray,
    translation_depth_to_color: np.ndarray,
    target_uv: tuple[float, float],
    color_mask: np.ndarray | None,
    radius_px: float = 7.0,
    min_depth_m: float = 0.08,
    max_depth_m: float = 0.80,
) -> tuple[np.ndarray, dict]:
    """Register raw D405 depth into color and robustly recover one 3-D point."""
    depth = np.asarray(depth_m, dtype=np.float64)
    if depth.shape != (depth_intr.height, depth_intr.width):
        raise ValueError("depth/intrinsics resolution mismatch")
    valid = np.isfinite(depth) & (depth >= min_depth_m) & (depth <= max_depth_m)
    v, u = np.nonzero(valid)
    if len(u) == 0:
        raise ValueError("no valid depth samples")
    z = depth[v, u]
    points_d = np.column_stack(
        ((u - depth_intr.cx) * z / depth_intr.fx,
         (v - depth_intr.cy) * z / depth_intr.fy,
         z)
    )
    rotation = np.asarray(rotation_depth_to_color, dtype=np.float64).reshape(3, 3)
    translation = np.asarray(translation_depth_to_color, dtype=np.float64).reshape(3)
    points_c = points_d @ rotation.T + translation
    positive = points_c[:, 2] > 1e-6
    points_c = points_c[positive]
    projected = np.column_stack(
        (color_intr.fx * points_c[:, 0] / points_c[:, 2] + color_intr.cx,
         color_intr.fy * points_c[:, 1] / points_c[:, 2] + color_intr.cy)
    )
    target = np.asarray(target_uv, dtype=np.float64)
    distance = np.linalg.norm(projected - target, axis=1)
    selected = distance <= float(radius_px)
    if color_mask is not None:
        rounded = np.rint(projected).astype(int)
        inside = (
            (rounded[:, 0] >= 0) & (rounded[:, 0] < color_intr.width)
            & (rounded[:, 1] >= 0) & (rounded[:, 1] < color_intr.height)
        )
        on_pad = np.zeros(len(rounded), dtype=bool)
        ids = np.flatnonzero(inside)
        on_pad[ids] = color_mask[rounded[ids, 1], rounded[ids, 0]] > 0
        selected &= on_pad
    candidates = points_c[selected]
    if len(candidates) < 6:
        raise ValueError(f"insufficient registered depth support ({len(candidates)})")

    # Reject a background surface at a dangling edge by selecting the dominant
    # 8 mm depth mode before taking the coordinate-wise median.
    bins = np.floor(candidates[:, 2] / 0.008).astype(int)
    values, counts = np.unique(bins, return_counts=True)
    dominant = values[int(np.argmax(counts))]
    cluster = candidates[np.abs(bins - dominant) <= 1]
    if len(cluster) < 6:
        raise ValueError("depth mode has insufficient support")
    point = np.median(cluster, axis=0)
    mad = float(np.median(np.abs(cluster[:, 2] - point[2])))
    if mad > 0.012:
        raise ValueError(f"depth spread too large ({mad:.6f} m)")
    return point, {
        "support": int(len(cluster)),
        "depth_median_m": float(point[2]),
        "depth_mad_m": mad,
    }


def quaternion_matrix(xyzw: list[float] | tuple[float, ...]) -> np.ndarray:
    x, y, z, w = np.asarray(xyzw, dtype=np.float64)
    norm = float(np.linalg.norm([x, y, z, w]))
    if norm < 1e-12:
        raise ValueError("zero quaternion")
    x, y, z, w = np.asarray([x, y, z, w]) / norm
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
    ])


def pose_matrix(position: list[float], quaternion_xyzw: list[float]) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = quaternion_matrix(quaternion_xyzw)
    result[:3, 3] = np.asarray(position, dtype=np.float64)
    return result


def transform_point(transform: np.ndarray, point: np.ndarray) -> np.ndarray:
    return (np.asarray(transform, dtype=np.float64) @ np.r_[point, 1.0])[:3]
