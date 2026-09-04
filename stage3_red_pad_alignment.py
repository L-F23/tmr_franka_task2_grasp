#!/usr/bin/env python3
"""Find the red pad, move the base to its station, then center wrist-image Y."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np

from base_motion import guarded_move_right, guarded_transport
from colored_pad_detector import (
    annotate,
    best_red_wrist_target,
    detect_colored_pads,
    estimate_red_station_from_reference,
    map_layout_to_distances,
)


ROOT = Path(__file__).resolve().parent
VIEWER = "http://127.0.0.1:18081"
# Current table calibration, supplied from the thermal-pad center.  The
# operator specified image-right to image-left as 52/41.5/29/16 cm, then
# requested cumulative uniform corrections totalling -2.7 cm.  Detector order
# is image-left to image-right, hence the reversed corrected values below.
KNOWN_DISTANCES_CM = [13.3, 26.3, 38.8, 49.3]
DEFAULT_CENTER_CALIBRATION = ROOT / "config" / "red_pad_center_calibration.json"
# Re-verified on the deployed table after the 2026-09-04 stopped run: positive
# right motion takes the robot from the black-base reference toward the pads.
# Distances from the visual fit remain unsigned.
RED_STATION_RIGHT_SIGN = 1.0
FAST_ALIGNMENT_SPEED_MPS = 0.04


def red_station_right_offset_m(distance_cm: float) -> float:
    return RED_STATION_RIGHT_SIGN * float(distance_cm) / 100.0


def frame(camera: str) -> np.ndarray:
    capture = cv2.VideoCapture(f"{VIEWER}/{camera}.mjpg")
    ok, image = capture.read()
    capture.release()
    if not ok or image is None:
        raise RuntimeError(f"{camera} camera unavailable")
    return image


def move_right(distance_m: float) -> dict:
    return guarded_move_right(distance_m, speed_mps=FAST_ALIGNMENT_SPEED_MPS)


def observe_wrist(center_tolerance_px: float) -> dict:
    image = frame("left")
    target = best_red_wrist_target(image)
    if target is None:
        return {"visible": False, "image_height": image.shape[0]}
    error = float(target.center_px[1] - image.shape[0] / 2.0)
    return {
        "visible": True,
        "center_px": list(target.center_px),
        "y_error_px": error,
        "centered": abs(error) <= center_tolerance_px,
        "confidence": target.confidence,
        "image_height": image.shape[0],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--from-black-base-reference", action="store_true",
        help="assert that current base position is the black-base grasp reference",
    )
    parser.add_argument("--center-tolerance-px", type=float, default=25.0)
    parser.add_argument("--probe-right-m", type=float, default=0.01)
    parser.add_argument(
        "--center-calibration", type=Path, default=DEFAULT_CENTER_CALIBRATION,
    )
    parser.add_argument(
        "--wrist-closed-loop", action="store_true",
        help="explicitly enable wrist-Y probe/correction; default is advisory-only",
    )
    parser.add_argument(
        "--record", type=Path, default=ROOT / "config" / "latest_stage3_red_pad.json"
    )
    args = parser.parse_args()

    calibration = json.loads(args.center_calibration.read_text(encoding="utf-8"))
    if calibration.get("schema_version") != 1:
        raise ValueError("unsupported red-pad center calibration schema")
    main_image = frame("main")
    main_detections = detect_colored_pads(
        main_image, roi=(0.55, 0.50, 0.96, 0.70), minimum_area_px=100.0
    )
    try:
        observed_layout = map_layout_to_distances(
            main_detections, KNOWN_DISTANCES_CM
        )
        ordered = sorted(main_detections, key=lambda item: item.center_px[0])
        live_anchors = [
            {
                "center_px": list(item.center_px),
                "distance_from_black_base_cm": float(distance),
            }
            for item, distance in zip(ordered, KNOWN_DISTANCES_CM)
        ]
        # The center model requires increasing physical distance; image order
        # is reversed at the present camera mounting, so sort independently.
        live_anchors.sort(key=lambda item: item["distance_from_black_base_cm"])
        live_calibration = {
            **calibration,
            "reference_image_size_px": [main_image.shape[1], main_image.shape[0]],
            "anchors": live_anchors,
        }
        red_station = estimate_red_station_from_reference(
            main_detections,
            live_calibration,
            (main_image.shape[1], main_image.shape[0]),
        )
        red_station["fit_source"] = "live_four_board_centers"
    except ValueError as exc:
        # All four boards are useful diagnostic context but are no longer
        # required at runtime. One reliable red center is sufficient for the
        # saved nonlinear center-to-distance calibration.
        observed_layout = {"status": "incomplete", "detail": str(exc)}
        red_station = estimate_red_station_from_reference(
            main_detections,
            calibration,
            (main_image.shape[1], main_image.shape[0]),
        )
        red_station["fit_source"] = "saved_reference_centers"
    raw_output = ROOT / "outputs" / "stage3_main_raw.jpg"
    raw_output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(raw_output), main_image)
    annotated = annotate(main_image, main_detections)
    output = ROOT / "outputs" / "stage3_main_layout.jpg"
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), annotated)
    report = {
        "status": "observed",
        "known_distances_from_thermal_pad_center_cm": KNOWN_DISTANCES_CM,
        "main_layout": observed_layout,
        "red_station_center_fit": red_station,
        "center_calibration": str(args.center_calibration),
        "main_raw_image": str(raw_output),
        "main_detections": [item.to_dict() for item in main_detections],
        "base_steps": [],
        "wrist_observations": [],
        "wrist_role": "closed_loop" if args.wrist_closed_loop else "advisory_only",
        "right_arm_commanded": False,
        "left_arm_commanded": False,
        "spine_commanded": False,
    }
    try:
        if not args.execute:
            report["status"] = "dry_run_valid"
            return_code = 0
        elif not args.from_black_base_reference:
            raise RuntimeError("--from-black-base-reference is required for coarse motion")
        else:
            coarse_right_m = red_station_right_offset_m(
                red_station["distance_from_black_base_cm"]
            )
            report["red_station_signed_right_m"] = coarse_right_m
            transport_steps, actual_right_m = guarded_transport(
                coarse_right_m,
                tolerance_m=0.005,
                maximum_steps=20,
                mover=move_right,
            )
            report["base_steps"].extend(transport_steps)
            report["coarse_transport"] = {
                "requested_right_m": coarse_right_m,
                "actual_right_m": actual_right_m,
                "residual_right_m": coarse_right_m - actual_right_m,
                "tolerance_m": 0.005,
                "control": "accumulated_odometry_with_terminal_residual_compensation",
            }
            time.sleep(0.5)
            observation = observe_wrist(args.center_tolerance_px)
            report["wrist_observations"].append(observation)
            if not args.wrist_closed_loop:
                report["status"] = "red_pad_coarse_station_reached"
                return_code = 0
            elif not observation["visible"]:
                raise RuntimeError(
                    "red pad not visible in left wrist after nominal station; base stopped"
                )
            elif not observation["centered"]:
                probe = float(args.probe_right_m)
                report["base_steps"].append(move_right(probe))
                time.sleep(0.5)
                probed = observe_wrist(args.center_tolerance_px)
                report["wrist_observations"].append(probed)
                if not probed["visible"]:
                    raise RuntimeError("red pad lost after wrist-axis probe; base stopped")
                jacobian = (probed["y_error_px"] - observation["y_error_px"]) / probe
                if abs(jacobian) < 100.0:
                    raise RuntimeError(f"wrist Y Jacobian too small: {jacobian:.3f} px/m")
                correction = float(np.clip(-probed["y_error_px"] / jacobian, -0.04, 0.04))
                if abs(correction) >= 0.008:
                    report["base_steps"].append(move_right(correction))
                    time.sleep(0.5)
                final = observe_wrist(args.center_tolerance_px)
                final["local_y_jacobian_px_per_right_m"] = jacobian
                report["wrist_observations"].append(final)
                if not final.get("centered", False):
                    raise RuntimeError(
                        f"red pad did not reach wrist Y center: {final}"
                    )
            if args.wrist_closed_loop and observation["visible"]:
                report["status"] = "red_pad_centered_under_left_wrist"
                return_code = 0
    except Exception as exc:
        report["status"] = "blocked"
        report["error"] = str(exc)
        return_code = 2
    finally:
        args.record.parent.mkdir(parents=True, exist_ok=True)
        args.record.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2), flush=True)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
