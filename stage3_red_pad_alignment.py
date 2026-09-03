#!/usr/bin/env python3
"""Find the red pad, move the base to its station, then center wrist-image Y."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import time

import cv2
import numpy as np

from colored_pad_detector import (
    annotate,
    best_red_wrist_target,
    detect_colored_pads,
    map_layout_to_distances,
)


ROOT = Path(__file__).resolve().parent
VIEWER = "http://127.0.0.1:18081"
BASE_HOST = "tmr-user@172.16.0.50"
REMOTE_MOVER = "/home/tmr-user/tmr_cycle/scripts/guarded_lateral_step.py"
BASE_ENV = (
    "source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
    "source /home/tmr-user/ros2_ws/install/setup.bash >/dev/null 2>&1 || true; "
    "export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 "
    "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp "
    "CYCLONEDDS_URI=file:///home/tmr-user/cyclonedds.xml"
)
KNOWN_DISTANCES_CM = [19.5, 32.9, 44.6, 58.0]


def frame(camera: str) -> np.ndarray:
    capture = cv2.VideoCapture(f"{VIEWER}/{camera}.mjpg")
    ok, image = capture.read()
    capture.release()
    if not ok or image is None:
        raise RuntimeError(f"{camera} camera unavailable")
    return image


def split_lateral_move(distance_m: float, maximum_step_m: float = 0.08) -> list[float]:
    if abs(distance_m) < 1e-9:
        return []
    direction = 1.0 if distance_m > 0.0 else -1.0
    remaining = abs(float(distance_m))
    steps = []
    while remaining > maximum_step_m:
        steps.append(direction * maximum_step_m)
        remaining -= maximum_step_m
    if remaining >= 0.008:
        steps.append(direction * remaining)
    elif steps:
        steps[-1] += direction * remaining
    else:
        steps.append(direction * 0.008)
    return steps


def move_right(distance_m: float) -> dict:
    command = (
        f"{BASE_ENV}; python3 {REMOTE_MOVER} --right-m {distance_m:.6f} "
        "--speed-mps 0.02 --timeout-s 15"
    )
    completed = subprocess.run(
        ["ssh", "-o", "BatchMode=yes", BASE_HOST, command],
        check=False, text=True, capture_output=True,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"guarded base step failed: {completed.stdout.strip()} "
            f"{completed.stderr.strip()}"
        )
    start = completed.stdout.find("{")
    return json.loads(completed.stdout[start:])


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
        "--wrist-closed-loop", action="store_true",
        help="explicitly enable wrist-Y probe/correction; default is advisory-only",
    )
    parser.add_argument(
        "--record", type=Path, default=ROOT / "config" / "latest_stage3_red_pad.json"
    )
    args = parser.parse_args()

    main_image = frame("main")
    main_detections = detect_colored_pads(
        main_image, roi=(0.55, 0.50, 0.96, 0.70), minimum_area_px=100.0
    )
    layout = map_layout_to_distances(main_detections, KNOWN_DISTANCES_CM)
    annotated = annotate(main_image, main_detections)
    output = ROOT / "outputs" / "stage3_main_layout.jpg"
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), annotated)
    report = {
        "status": "observed",
        "known_distances_from_black_base_cm": KNOWN_DISTANCES_CM,
        "main_layout": layout,
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
            coarse_right_m = float(layout["red_station_distance_cm"]) / 100.0
            for step in split_lateral_move(coarse_right_m):
                report["base_steps"].append(move_right(step))
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
