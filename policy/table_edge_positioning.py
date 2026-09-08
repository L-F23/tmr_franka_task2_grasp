#!/usr/bin/env python3
"""Calibrate and align table distance from main and left-wrist RGB edges."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np

from base_motion import guarded_move_forward


ROOT = Path(__file__).resolve().parent
VIEWER = "http://127.0.0.1:18081"
DEFAULT_CONFIG = ROOT / "config" / "table_edge_reference.json"
DEFAULT_RECORD = ROOT / "config" / "latest_table_edge_alignment.json"


def frame(name: str) -> np.ndarray:
    capture = cv2.VideoCapture(f"{VIEWER}/{name}.mjpg")
    ok, image = capture.read()
    capture.release()
    if not ok or image is None:
        raise RuntimeError(f"{name} camera unavailable")
    return image


def main_table_edges(image: np.ndarray) -> dict:
    height, width = image.shape[:2]
    x0, x1 = int(0.55 * width), int(0.90 * width)
    gray = cv2.cvtColor(image[:, x0:x1], cv2.COLOR_BGR2GRAY)
    row = cv2.GaussianBlur(
        gray.mean(axis=1).astype(np.float32).reshape(-1, 1), (1, 15), 0
    ).ravel()
    gradient = np.gradient(row)
    lo, hi = int(0.28 * height), int(0.55 * height)
    top = int(lo + np.argmax(gradient[lo:hi]))
    hsv = cv2.cvtColor(image[:, x0:x1], cv2.COLOR_BGR2HSV)
    table_fraction = ((hsv[:, :, 1] < 75) & (hsv[:, :, 2] > 105)).mean(axis=1)
    candidates = np.flatnonzero(table_fraction > 0.60)
    bottom = int(candidates[-1]) if candidates.size else height - 1
    if gradient[top] < 2.0 or bottom <= top + int(0.20 * height):
        raise RuntimeError("main-camera table top/bottom edges are not reliable")
    return {
        "top_y_px": top,
        "bottom_y_px": bottom,
        "top_gradient": float(gradient[top]),
        "roi_x_px": [x0, x1],
    }


def wrist_left_table_edge(image: np.ndarray) -> dict:
    height, width = image.shape[:2]
    y0, y1 = int(0.10 * height), int(0.52 * height)
    gray = cv2.cvtColor(image[y0:y1], cv2.COLOR_BGR2GRAY)
    # The edge is perspective-slanted.  Averaging columns smears it and also
    # loses it after lateral alignment moves the line left of the old ROI.
    # Detect it independently per row, then reject outliers robustly.
    lo, hi = int(0.05 * width), int(0.42 * width)
    locations: list[int] = []
    strengths: list[float] = []
    for row in gray:
        smooth = cv2.GaussianBlur(
            row.astype(np.float32).reshape(1, -1), (15, 1), 0
        ).ravel()
        gradient = np.gradient(smooth)
        left = int(lo + np.argmax(gradient[lo:hi]))
        if gradient[left] >= 1.5:
            locations.append(left)
            strengths.append(float(gradient[left]))
    if len(locations) < max(25, int(0.25 * (y1 - y0))):
        raise RuntimeError("left-wrist table edge is not reliable")
    left = int(round(float(np.median(locations))))
    return {
        "left_x_px": left,
        "left_gradient": float(np.median(strengths)),
        "roi_y_px": [y0, y1],
    }


def features(main: np.ndarray, left: np.ndarray) -> dict:
    return {
        "main": main_table_edges(main),
        "left_wrist": wrist_left_table_edge(left),
        "image_shapes": {
            "main": list(main.shape),
            "left": list(left.shape),
        },
    }


def decision(current: dict, reference: dict) -> dict:
    main_error = float(current["main"]["top_y_px"] - reference["main"]["top_y_px"])
    wrist_error = float(
        current["left_wrist"]["left_x_px"] - reference["left_wrist"]["left_x_px"]
    )
    main_tolerance = float(reference["tolerances_px"]["main_top_y"])
    wrist_tolerance = float(reference["tolerances_px"]["left_wrist_left_x"])
    votes = []
    if abs(main_error) > main_tolerance:
        # Moving forward shifts the main-camera table top downward (larger Y),
        # so a positive error must be corrected by moving backward.
        votes.append(-1 if main_error > 0 else 1)
    if abs(wrist_error) > wrist_tolerance:
        votes.append(1 if wrist_error > 0 else -1)
    if votes and min(votes) != max(votes):
        raise RuntimeError(
            f"main/wrist table-distance cues disagree: main={main_error:+.1f}px, "
            f"wrist={wrist_error:+.1f}px"
        )
    command = "aligned" if not votes else ("forward" if votes[0] > 0 else "backward")
    return {
        "decision": command,
        "main_top_error_px": main_error,
        "left_wrist_edge_error_px": wrist_error,
    }


def annotate(image: np.ndarray, axis: str, value: int) -> np.ndarray:
    output = image.copy()
    if axis == "y":
        cv2.line(output, (0, value), (output.shape[1] - 1, value), (0, 0, 255), 2)
    else:
        cv2.line(output, (value, 0), (value, output.shape[0] - 1), (0, 0, 255), 2)
    return output


def calibrate(args) -> int:
    main = cv2.imread(str(args.main_image)) if args.main_image else frame("main")
    left = cv2.imread(str(args.left_image)) if args.left_image else frame("left")
    if main is None or left is None:
        raise RuntimeError("calibration images could not be read")
    observed = features(main, left)
    config = {
        "schema_version": 1,
        "calibrated_at_unix_s": time.time(),
        "semantics": "current operator-confirmed table-edge grasp distance",
        **observed,
        "tolerances_px": {"main_top_y": 12, "left_wrist_left_x": 12},
        "step_m": 0.01,
        "maximum_steps": 24,
        "direction_model": {
            "main_positive_y_error": "base_backward",
            "left_wrist_positive_x_error": "base_forward",
        },
        "source_images": {
            "main": str(args.main_image) if args.main_image else "live",
            "left": str(args.left_image) if args.left_image else "live",
        },
    }
    args.config.parent.mkdir(parents=True, exist_ok=True)
    args.config.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    output = ROOT / "outputs"
    output.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output / "table_edge_main_reference.jpg"), annotate(main, "y", observed["main"]["top_y_px"]))
    cv2.imwrite(str(output / "table_edge_left_reference.jpg"), annotate(left, "x", observed["left_wrist"]["left_x_px"]))
    print(json.dumps(config, indent=2), flush=True)
    return 0


def align(args) -> int:
    reference = json.loads(args.config.read_text(encoding="utf-8"))
    history = []
    result = {"status": "blocked", "base_steps": [], "physical_motion_commanded": False}
    try:
        consecutive = 0
        for _ in range(int(reference["maximum_steps"]) + 2):
            current = features(frame("main"), frame("left"))
            state = decision(current, reference)
            state["features"] = current
            history.append(state)
            print(json.dumps(state), flush=True)
            if state["decision"] == "aligned":
                consecutive += 1
                if consecutive >= 2:
                    result["status"] = "table_distance_aligned"
                    break
                time.sleep(0.25)
                continue
            consecutive = 0
            if not args.execute:
                result["status"] = "adjustment_required"
                break
            signed_step = float(reference["step_m"])
            if state["decision"] == "backward":
                signed_step = -signed_step
            result["base_steps"].append(guarded_move_forward(signed_step, speed_mps=0.015))
            result["physical_motion_commanded"] = True
            time.sleep(0.4)
        else:
            raise RuntimeError("table-distance alignment step limit reached")
        result["history"] = history
        result["reference"] = str(args.config)
        return_code = 0 if result["status"] == "table_distance_aligned" else 2
    except Exception as exc:
        result["error"] = str(exc)
        result["history"] = history
        return_code = 2
    finally:
        args.record.parent.mkdir(parents=True, exist_ok=True)
        args.record.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, indent=2), flush=True)
    return return_code


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibrate", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--main-image", type=Path)
    parser.add_argument("--left-image", type=Path)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--record", type=Path, default=DEFAULT_RECORD)
    args = parser.parse_args()
    return calibrate(args) if args.calibrate else align(args)


if __name__ == "__main__":
    raise SystemExit(main())
