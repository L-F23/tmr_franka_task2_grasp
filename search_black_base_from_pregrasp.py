#!/usr/bin/env python3
"""Search right for the saved pregrasp black-base view.

The optional initial left displacement supports standalone calibration runs;
the CCW-restore pipeline sets it to zero and starts searching from its fixed
pre-search offset.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np

from alignment_detector import detect_target
from base_motion import guarded_move_right
from mission_runtime import (
    MissionAlreadyRunning,
    acquire_motion_lock,
    atomic_write_json,
    release_motion_lock,
)


ROOT = Path(__file__).resolve().parent
VIEWER = "http://127.0.0.1:18081"
DEFAULT_RECORD = ROOT / "config" / "latest_pregrasp_black_base_search.json"
DEFAULT_REFERENCE = ROOT / "captures" / "pregrasp_black_base_reference_left.jpg"
DEFAULT_ROI = (290, 160, 250, 155)


def frame() -> np.ndarray:
    capture = cv2.VideoCapture(f"{VIEWER}/left.mjpg")
    ok, image = capture.read()
    capture.release()
    if not ok or image is None:
        raise RuntimeError("left wrist camera unavailable")
    return image


def observe_reference(
    image: np.ndarray,
    reference: np.ndarray,
    bbox_xywh: tuple[int, int, int, int],
    minimum_confidence: float,
    center_tolerance_px: float,
) -> dict:
    x, y, width, height = bbox_xywh
    template = cv2.cvtColor(reference[y:y + height, x:x + width], cv2.COLOR_BGR2GRAY)
    if template.size == 0:
        raise RuntimeError("pregrasp reference ROI is empty")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    response = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
    _, confidence, _, location = cv2.minMaxLoc(response)
    center = [location[0] + width / 2.0, location[1] + height / 2.0]
    reference_center = [x + width / 2.0, y + height / 2.0]
    error = [center[0] - reference_center[0], center[1] - reference_center[1]]
    visible = confidence >= minimum_confidence
    return {
        "visible": visible,
        "near_reference": bool(
            visible
            and abs(error[0]) <= center_tolerance_px
            and abs(error[1]) <= center_tolerance_px
        ),
        "confidence": float(confidence),
        "center_px": center,
        "reference_center_px": reference_center,
        "center_error_px": error,
    }


def command_right_step(observation: dict, coarse_step_m: float) -> float:
    """Use right-only coarse search, then reduce the step once the target is visible."""
    if not observation["visible"]:
        return coarse_step_m
    y_error = float(observation["center_error_px"][1])
    if y_error > 0.0:
        raise RuntimeError(
            "target passed the reference in the rightward search; refusing further right motion"
        )
    # The measured wrist mapping is about 2.2 px/mm.  A 20 mm request is small
    # enough to enter the reference tolerance without skipping over it.
    return min(0.020, coarse_step_m)


def visible_target(image: np.ndarray) -> dict | None:
    """Return structured left-wrist evidence without requiring reference scale."""
    target = detect_target(image)
    if target is None:
        return None
    return {
        "detector": "complete_black_base_and_grey_pad",
        "center_px": list(target.center),
        "bbox_xywh": list(target.base_box),
        "area_px": float(target.area),
        "confidence": float(target.confidence),
    }


def move_left_one_metre(report: dict, distance_m: float) -> None:
    completed = 0.0
    index = 0
    while abs(completed) < distance_m - 0.005:
        remaining = distance_m - abs(completed)
        request = -min(0.08, max(0.008, remaining))
        result = guarded_move_right(request, speed_mps=0.04)
        completed += float(result["actual_right_m"])
        index += 1
        item = dict(result, index=index, cumulative_right_m=completed)
        report["initial_left_steps"].append(item)
        print(json.dumps({"phase": "leave_reference_left", **item}), flush=True)
    report["actual_initial_left_m"] = completed


def execute(args: argparse.Namespace) -> int:
    if args.use_existing_reference:
        reference = cv2.imread(str(args.reference))
        if reference is None:
            raise RuntimeError("existing pregrasp wrist reference is unavailable")
    else:
        reference = frame()
        args.reference.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(args.reference), reference):
            raise RuntimeError("failed to save pregrasp wrist reference")
    report = {
        "schema_version": 1,
        "status": "running",
        "started_at_unix_s": time.time(),
        "entry_condition": "left arm held at calibrated pregrasp pose",
        "initial_reset_stage_removed": True,
        "reference_image": str(args.reference),
        "reference_bbox_xywh": list(DEFAULT_ROI),
        "maximum_right_search_m": args.maximum_right_m,
        "initial_left_steps": [],
        "right_search_steps": [],
        "observations": [],
        "left_arm_commanded": False,
        "right_arm_commanded": False,
        "spine_commanded": False,
    }
    atomic_write_json(args.record, report)
    if args.initial_left_m > 0.0:
        move_left_one_metre(report, args.initial_left_m)
    else:
        report["actual_initial_left_m"] = 0.0
    atomic_write_json(args.record, report)

    actual_right = 0.0
    consecutive = 0
    while actual_right < args.maximum_right_m - 0.005:
        image = frame()
        observation = observe_reference(
            image, reference, DEFAULT_ROI,
            args.minimum_confidence, args.center_tolerance_px,
        )
        observation["structured_target"] = visible_target(image)
        observation.update(
            observed_at_unix_s=time.time(), cumulative_actual_right_m=actual_right
        )
        report["observations"].append(observation)
        print(json.dumps({"phase": "search_observation", **observation}), flush=True)
        target_seen = (
            args.stop_on_target_visible
            and observation["structured_target"] is not None
        )
        if observation["near_reference"] or target_seen:
            consecutive += 1
            if consecutive >= 2:
                report.update(
                    status=(
                        "pregrasp_black_base_target_visible"
                        if target_seen and not observation["near_reference"]
                        else "pregrasp_black_base_reference_reacquired"
                    ),
                    actual_right_search_m=actual_right,
                    net_right_from_calibrated_reference_m=(
                        float(report["actual_initial_left_m"]) + actual_right
                    ),
                    final_observation=observation,
                    finished_at_unix_s=time.time(),
                )
                atomic_write_json(args.record, report)
                print(json.dumps(report, indent=2), flush=True)
                return 0
            time.sleep(0.2)
            continue
        consecutive = 0
        requested = command_right_step(observation, args.coarse_step_m)
        remaining = args.maximum_right_m - actual_right
        if remaining < 0.008:
            break
        requested = min(requested, remaining)
        result = guarded_move_right(requested, speed_mps=0.04)
        actual_right += float(result["actual_right_m"])
        item = dict(
            result,
            index=len(report["right_search_steps"]) + 1,
            cumulative_right_m=actual_right,
        )
        report["right_search_steps"].append(item)
        print(json.dumps({"phase": "search_right", **item}), flush=True)
        atomic_write_json(args.record, report)

    report.update(
        status="maximum_right_search_reached_without_reference",
        actual_right_search_m=actual_right,
        net_right_from_calibrated_reference_m=(
            float(report["actual_initial_left_m"]) + actual_right
        ),
        finished_at_unix_s=time.time(),
    )
    atomic_write_json(args.record, report)
    print(json.dumps(report, indent=2), flush=True)
    return 2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--initial-left-m", type=float, default=1.0)
    parser.add_argument("--maximum-right-m", type=float, default=2.0)
    parser.add_argument("--coarse-step-m", type=float, default=0.08)
    parser.add_argument("--minimum-confidence", type=float, default=0.72)
    parser.add_argument("--center-tolerance-px", type=float, default=20.0)
    parser.add_argument("--reference", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument("--use-existing-reference", action="store_true")
    parser.add_argument("--stop-on-target-visible", action="store_true")
    parser.add_argument("--record", type=Path, default=DEFAULT_RECORD)
    args = parser.parse_args()
    if not args.execute:
        print(json.dumps({
            "status": "dry_run",
            "physical_motion_commanded": False,
            "ordered_phases": [
                "capture_current_pregrasp_reference",
                "move_base_left_1m",
                "search_base_right_until_reference_or_2m",
            ],
        }, indent=2))
        return 0
    if args.initial_left_m != 0.0 and not 0.08 <= args.initial_left_m <= 1.0:
        parser.error("--initial-left-m must be 0 or in [0.08, 1.0]")
    if not 0.08 <= args.maximum_right_m <= 2.0:
        parser.error("--maximum-right-m must be in [0.08, 2.0]")
    if not 0.02 <= args.coarse_step_m <= 0.08:
        parser.error("--coarse-step-m must be in [0.02, 0.08]")

    lock = None
    try:
        lock = acquire_motion_lock()
        return execute(args)
    except MissionAlreadyRunning as exc:
        print(json.dumps({"status": "blocked", "error": str(exc)}, indent=2))
        return 73
    except Exception as exc:
        print(json.dumps({"status": "blocked", "error": str(exc)}, indent=2))
        return 2
    finally:
        release_motion_lock(lock)


if __name__ == "__main__":
    raise SystemExit(main())
