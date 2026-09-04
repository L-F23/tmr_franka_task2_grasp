#!/usr/bin/env python3
"""Calibrate left-wrist image-Y error to base lateral correction at pregrasp."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np

from base_motion import guarded_move_right, guarded_transport
from thermal_pad_ik import ROOT


VIEWER = "http://127.0.0.1:18081"
DEFAULT_CONFIG = ROOT / "config" / "wrist_lateral_mapping.json"
DEFAULT_RECORD = ROOT / "config" / "latest_wrist_lateral_mapping.json"
DEFAULT_CAPTURE_DIR = ROOT / "captures" / "wrist_lateral_mapping_20260904"
REFERENCE_BBOX_XYWH = (290, 160, 250, 155)


def frame() -> np.ndarray:
    capture = cv2.VideoCapture(f"{VIEWER}/left.mjpg")
    ok, image = capture.read()
    capture.release()
    if not ok or image is None:
        raise RuntimeError("left wrist camera unavailable")
    return image


def match_template(image: np.ndarray, template_gray: np.ndarray) -> dict:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    response = cv2.matchTemplate(gray, template_gray, cv2.TM_CCOEFF_NORMED)
    _, confidence, _, location = cv2.minMaxLoc(response)
    height, width = template_gray.shape
    return {
        "top_left_px": [int(location[0]), int(location[1])],
        "center_px": [
            float(location[0] + width / 2.0),
            float(location[1] + height / 2.0),
        ],
        "confidence": float(confidence),
    }


def fit_mapping(samples: list[dict], reference_y_px: float) -> dict:
    errors = np.asarray(
        [sample["median_center_y_px"] - reference_y_px for sample in samples],
        dtype=np.float64,
    )
    offsets = np.asarray(
        [sample["cumulative_actual_right_m"] for sample in samples],
        dtype=np.float64,
    )
    corrections = -offsets
    usable = np.abs(errors) >= 0.25
    if int(np.count_nonzero(usable)) < 4:
        raise RuntimeError("insufficient wrist target motion for calibration")
    design = np.column_stack((errors[usable], errors[usable] ** 2))
    inverse_coefficients, *_ = np.linalg.lstsq(design, corrections[usable], rcond=None)
    predicted = np.column_stack((errors, errors ** 2)) @ inverse_coefficients
    inverse_rmse = float(np.sqrt(np.mean((predicted - corrections) ** 2)))

    forward_design = np.column_stack((offsets[usable], offsets[usable] ** 2))
    forward_coefficients, *_ = np.linalg.lstsq(
        forward_design, errors[usable], rcond=None
    )
    forward_predicted = (
        np.column_stack((offsets, offsets ** 2)) @ forward_coefficients
    )
    forward_rmse = float(np.sqrt(np.mean((forward_predicted - errors) ** 2)))
    return {
        "model": "zero_anchored_quadratic",
        "input": "left_wrist_target_center_y_error_px",
        "output": "commanded_base_right_correction_m",
        "right_correction_m": {
            "linear_m_per_px": float(inverse_coefficients[0]),
            "quadratic_m_per_px2": float(inverse_coefficients[1]),
            "rmse_m": inverse_rmse,
        },
        "forward_diagnostic": {
            "linear_px_per_m_right": float(forward_coefficients[0]),
            "quadratic_px_per_m2_right": float(forward_coefficients[1]),
            "rmse_px": forward_rmse,
        },
        "direction_semantics": {
            "negative_correction": "move_base_left",
            "positive_correction": "move_base_right",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--probe-command-m", type=float, default=0.02)
    parser.add_argument("--frames-per-sample", type=int, default=5)
    parser.add_argument("--minimum-confidence", type=float, default=0.60)
    parser.add_argument("--speed-mps", type=float, default=0.012)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--record", type=Path, default=DEFAULT_RECORD)
    parser.add_argument("--capture-dir", type=Path, default=DEFAULT_CAPTURE_DIR)
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required for physical calibration")
    if not 0.008 <= args.probe_command_m <= 0.03:
        parser.error("--probe-command-m must be in [0.008, 0.03]")
    if not 3 <= args.frames_per_sample <= 15:
        parser.error("--frames-per-sample must be in [3, 15]")

    args.capture_dir.mkdir(parents=True, exist_ok=True)
    reference = frame()
    x, y, width, height = REFERENCE_BBOX_XYWH
    template = cv2.cvtColor(
        reference[y:y + height, x:x + width], cv2.COLOR_BGR2GRAY
    )
    if template.size == 0:
        raise RuntimeError("invalid wrist reference template")
    reference_path = args.capture_dir / "zero_left.jpg"
    cv2.imwrite(str(reference_path), reference)
    reference_center = [x + width / 2.0, y + height / 2.0]
    report = {
        "schema_version": 1,
        "status": "running",
        "started_at_unix_s": time.time(),
        "fixed_pose": "left_stage1_pregrasp_start",
        "reference_is_operator_confirmed_aligned": True,
        "reference_image": str(reference_path.relative_to(ROOT)),
        "reference_bbox_xywh": list(REFERENCE_BBOX_XYWH),
        "reference_target_center_px": reference_center,
        "samples": [],
        "base_steps": [],
        "left_arm_commanded": False,
        "right_arm_commanded": False,
        "spine_commanded": False,
    }
    cumulative_right_m = 0.0

    def sample(index: int) -> None:
        observations = []
        images = []
        for _ in range(args.frames_per_sample):
            image = frame()
            images.append(image)
            observations.append(match_template(image, template))
            time.sleep(0.08)
        confidences = np.asarray(
            [observation["confidence"] for observation in observations]
        )
        if float(np.median(confidences)) < args.minimum_confidence:
            raise RuntimeError(
                f"wrist template confidence too low: {np.median(confidences):.3f}"
            )
        center_y = np.asarray(
            [observation["center_px"][1] for observation in observations]
        )
        image_path = args.capture_dir / f"sample_{index:02d}.jpg"
        cv2.imwrite(str(image_path), images[len(images) // 2])
        item = {
            "index": index,
            "cumulative_actual_right_m": cumulative_right_m,
            "median_center_y_px": float(np.median(center_y)),
            "median_center_y_error_px": float(
                np.median(center_y) - reference_center[1]
            ),
            "median_confidence": float(np.median(confidences)),
            "peak_to_peak_center_y_px": float(np.ptp(center_y)),
            "image": str(image_path.relative_to(ROOT)),
        }
        report["samples"].append(item)
        print(json.dumps({"wrist_sample": item}), flush=True)

    try:
        sample(0)
        # Visit both sides and the center twice.  Actual odometry, rather than
        # nominal command distance, is used by the fit.
        command_signs = (-1, -1, 1, 1, 1, 1, -1, -1)
        for index, sign in enumerate(command_signs, start=1):
            result = guarded_move_right(
                sign * args.probe_command_m,
                speed_mps=args.speed_mps,
                timeout_s=15.0,
            )
            actual = float(result["actual_right_m"])
            cumulative_right_m += actual
            report["base_steps"].append({
                **result,
                "index": index,
                "cumulative_actual_right_m": cumulative_right_m,
            })
            time.sleep(0.25)
            sample(index)

        if abs(cumulative_right_m) >= 0.008:
            return_steps, returned = guarded_transport(
                -cumulative_right_m,
                tolerance_m=0.002,
                maximum_steps=8,
                mover=lambda distance: guarded_move_right(
                    distance, speed_mps=args.speed_mps, timeout_s=15.0
                ),
            )
            cumulative_right_m += returned
            report["return_steps"] = return_steps
        report["return_residual_right_m"] = cumulative_right_m
        if abs(cumulative_right_m) > 0.004:
            raise RuntimeError(
                f"failed to return to aligned zero: {cumulative_right_m:.6f} m"
            )

        mapping = fit_mapping(report["samples"], reference_center[1])
        config = {
            "schema_version": 1,
            "fixed_pose": report["fixed_pose"],
            "reference_image": report["reference_image"],
            "reference_bbox_xywh": report["reference_bbox_xywh"],
            "reference_target_center_y_px": reference_center[1],
            "minimum_template_confidence": args.minimum_confidence,
            "valid_center_y_error_px": [
                float(min(s["median_center_y_error_px"] for s in report["samples"])),
                float(max(s["median_center_y_error_px"] for s in report["samples"])),
            ],
            "mapping": mapping,
        }
        args.config.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
        report["mapping"] = mapping
        report["config"] = str(args.config)
        report["status"] = "calibration_complete_returned_to_zero"
        report["finished_at_unix_s"] = time.time()
        code = 0
    except Exception as exc:
        report["status"] = "failed"
        report["error"] = str(exc)
        report["base_offset_at_failure_m"] = cumulative_right_m
        code = 2
    finally:
        args.record.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2), flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
