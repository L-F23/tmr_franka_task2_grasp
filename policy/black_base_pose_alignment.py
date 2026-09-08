#!/usr/bin/env python3
"""Align the black thermal-pad stand at the fixed left-arm pregrasp pose.

The grey sheet may be almost edge-on.  This gate therefore matches three
overlapping regions of the rigid black stand, jointly estimates image scale
(fore/aft error) and vertical image displacement (base lateral error), and
does not use the sheet appearance as the alignment reference.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np

from base_motion import guarded_move_forward, guarded_move_right
from mission_runtime import atomic_write_json
from pregrasp_lateral_alignment import open_gripper
from thermal_pad_ik import DEFAULT_CONFIG, ROOT


VIEWER = "http://127.0.0.1:18081"
DEFAULT_MAPPING = ROOT / "config" / "wrist_lateral_mapping.json"
DEFAULT_RECORD = ROOT / "config" / "latest_pregrasp_lateral_alignment.json"
BASE_ROIS = (
    (340, 125, 270, 220),
    (360, 130, 240, 210),
    (390, 120, 210, 230),
)


def frame() -> np.ndarray:
    capture = cv2.VideoCapture(f"{VIEWER}/left.mjpg")
    ok, image = capture.read()
    capture.release()
    if not ok or image is None:
        raise RuntimeError("left wrist camera unavailable")
    return image


def match_base_structure(
    image: np.ndarray,
    reference: np.ndarray,
    *,
    scale_low: float = 0.60,
    scale_high: float = 1.10,
    scale_count: int = 26,
) -> dict:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    matches = []
    for x, y, width, height in BASE_ROIS:
        raw = cv2.cvtColor(
            reference[y:y + height, x:x + width], cv2.COLOR_BGR2GRAY
        )
        best = None
        for scale in np.linspace(scale_low, scale_high, scale_count):
            template = cv2.resize(
                raw,
                None,
                fx=float(scale),
                fy=float(scale),
                interpolation=(
                    cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
                ),
            )
            _, confidence, _, location = cv2.minMaxLoc(cv2.matchTemplate(
                gray, template, cv2.TM_CCOEFF_NORMED
            ))
            candidate = (
                float(confidence), float(scale), location,
                template.shape[1], template.shape[0],
            )
            if best is None or candidate[0] > best[0]:
                best = candidate
        confidence, scale, location, matched_width, matched_height = best
        matches.append({
            "reference_roi_xywh": [x, y, width, height],
            "confidence": confidence,
            "scale": scale,
            "center_error_px": [
                location[0] + matched_width / 2.0 - (x + width / 2.0),
                location[1] + matched_height / 2.0 - (y + height / 2.0),
            ],
        })
    scales = [item["scale"] for item in matches]
    y_errors = [item["center_error_px"][1] for item in matches]
    return {
        "matches": matches,
        "minimum_confidence": min(item["confidence"] for item in matches),
        "median_scale": float(np.median(scales)),
        "scale_spread": float(max(scales) - min(scales)),
        "median_center_y_error_px": float(np.median(y_errors)),
        "center_y_error_spread_px": float(max(y_errors) - min(y_errors)),
    }


def require_consensus(observation: dict) -> None:
    if observation["minimum_confidence"] < 0.82:
        raise RuntimeError("black-base structure confidence is too low")
    if observation["scale_spread"] > 0.08:
        raise RuntimeError("black-base templates disagree on depth scale")
    if observation["center_y_error_spread_px"] > 12.0:
        raise RuntimeError("black-base templates disagree on lateral position")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--mapping", type=Path, default=DEFAULT_MAPPING)
    parser.add_argument("--robot-config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--record", type=Path, default=DEFAULT_RECORD)
    parser.add_argument("--maximum-depth-steps", type=int, default=5)
    parser.add_argument("--maximum-lateral-steps", type=int, default=5)
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required for physical alignment")

    mapping = json.loads(args.mapping.read_text(encoding="utf-8"))
    robot_config = json.loads(args.robot_config.read_text(encoding="utf-8"))
    reference = cv2.imread(str(ROOT / mapping["reference_image"]))
    if reference is None:
        raise RuntimeError("black-base alignment reference image unavailable")
    report = {
        "status": "blocked",
        "alignment_source": "left_wrist_black_base_multiscale_consensus",
        "grey_sheet_appearance_used": False,
        "history": [],
        "base_steps": [],
        "right_arm_commanded": False,
        "left_arm_commanded": False,
        "spine_commanded": False,
    }
    code = 2
    try:
        report["gripper_open"] = open_gripper(robot_config)

        for _ in range(args.maximum_depth_steps + 1):
            observation = match_base_structure(frame(), reference)
            observation["phase"] = "fore_aft_scale"
            require_consensus(observation)
            report["history"].append(observation)
            print(json.dumps(observation), flush=True)
            scale = float(observation["median_scale"])
            if 0.94 <= scale <= 1.04:
                break
            if len([x for x in report["history"] if x["phase"] == "fore_aft_scale"]) > args.maximum_depth_steps:
                raise RuntimeError("black-base depth alignment step limit reached")
            requested = 0.020 if scale < 0.94 else -0.020
            result = guarded_move_forward(requested, speed_mps=0.015)
            report["base_steps"].append({"axis": "forward", **result})
            time.sleep(0.30)

        for _ in range(args.maximum_lateral_steps + 1):
            observation = match_base_structure(
                frame(), reference,
                scale_low=0.85, scale_high=1.08, scale_count=24,
            )
            observation["phase"] = "lateral_center"
            require_consensus(observation)
            report["history"].append(observation)
            print(json.dumps(observation), flush=True)
            error = float(observation["median_center_y_error_px"])
            if abs(error) <= 7.0 and 0.94 <= observation["median_scale"] <= 1.04:
                report["status"] = "pregrasp_lateral_alignment_confirmed"
                report["aligned_at_unix_s"] = time.time()
                report["final_observation"] = observation
                code = 0
                break
            if len([x for x in report["history"] if x["phase"] == "lateral_center"]) > args.maximum_lateral_steps:
                raise RuntimeError("black-base lateral alignment step limit reached")
            magnitude = min(0.020, max(0.008, abs(error) / 2200.0))
            requested = -float(np.sign(error)) * magnitude
            result = guarded_move_right(requested, speed_mps=0.015)
            report["base_steps"].append({"axis": "right", **result})
            time.sleep(0.30)
    except Exception as exc:
        report["error"] = str(exc)
    finally:
        atomic_write_json(args.record, report)
        print(json.dumps(report, indent=2), flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
