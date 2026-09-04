#!/usr/bin/env python3
"""CCW 90°, align heading to the rear wall, then request confirmation.

The command-line entry remains dry-run by default; ``--execute`` is required
for physical rotation.  Its base-controller bridge is also reused by the
validated CCW-restore pipeline.  The standalone mode can defer fore/aft wall
distance correction and the later rightward ZED search for operator review.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import time
from urllib.request import ProxyHandler, build_opener

import cv2
import numpy as np

from alignment_detector import detect_target
from base_motion import (
    BASE_ENV,
    BASE_HOST,
    _extract_last_json_object,
    guarded_move_right,
)
from mission_runtime import (
    MissionAlreadyRunning,
    acquire_motion_lock,
    atomic_write_json,
    release_motion_lock,
)


ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = ROOT / "config" / "wall_dock_search.json"
DEFAULT_RECORD = ROOT / "config" / "latest_wall_dock_search.json"
BASE_CONTROLLER = ROOT / "stage0_wall_docking_base.py"
DIRECT_OPENER = build_opener(ProxyHandler({}))


def detect_black_stand(image: np.ndarray) -> dict | None:
    """Detect a dark compact stand on the bright tabletop in the ZED image."""
    if not isinstance(image, np.ndarray) or image.ndim != 3:
        raise ValueError("image must be a BGR array")
    height, width = image.shape[:2]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    value, saturation = hsv[:, :, 2], hsv[:, :, 1]
    dark = np.uint8((value <= 75) & (saturation <= 180)) * 255
    allowed = np.zeros_like(dark)
    # During this rightward search the stand first enters from the image's
    # right side. The left/centre foreground is occupied by the robot itself;
    # accepting dark blobs there misclassifies the black wrist/joint housing.
    allowed[
        int(0.28 * height):int(0.90 * height),
        int(0.56 * width):int(0.97 * width),
    ] = 255
    mask = cv2.bitwise_and(dark, allowed)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        x, y, box_width, box_height = cv2.boundingRect(contour)
        if not 0.0015 * width * height <= area <= 0.12 * width * height:
            continue
        if box_width < 28 or box_height < 22:
            continue
        aspect = box_width / max(1.0, float(box_height))
        if not 1.10 <= aspect <= 4.0:
            continue
        rectangularity = area / max(1.0, float(box_width * box_height))
        if rectangularity < 0.42:
            continue
        margin = max(12, int(0.35 * max(box_width, box_height)))
        x0, x1 = max(0, x - margin), min(width, x + box_width + margin)
        y0, y1 = max(0, y - margin), min(height, y + box_height + margin)
        ring = np.ones((y1 - y0, x1 - x0), dtype=bool)
        ring[y - y0:y + box_height - y0, x - x0:x + box_width - x0] = False
        ring_value = value[y0:y1, x0:x1][ring]
        ring_saturation = saturation[y0:y1, x0:x1][ring]
        if ring_value.size == 0:
            continue
        bright_table_fraction = float(np.mean(
            (ring_value >= 120) & (ring_saturation <= 100)
        ))
        if bright_table_fraction < 0.28:
            continue
        confidence = float(np.clip(
            0.45 * rectangularity
            + 0.35 * bright_table_fraction
            + 0.20 * min(1.0, area / 3000.0),
            0.0,
            1.0,
        ))
        candidates.append({
            "center_px": [x + box_width / 2.0, y + box_height / 2.0],
            "bbox_xywh": [x, y, box_width, box_height],
            "area_px": area,
            "rectangularity": rectangularity,
            "bright_table_ring_fraction": bright_table_fraction,
            "confidence": confidence,
            "detector": "black_stand_only",
        })
    return max(candidates, key=lambda item: item["confidence"], default=None)


def observe_black_stand(image: np.ndarray) -> dict | None:
    structured = detect_target(image)
    if structured is not None:
        return {
            "center_px": list(structured.center),
            "bbox_xywh": list(structured.base_box),
            "area_px": structured.area,
            "confidence": structured.confidence,
            "detector": "black_stand_with_grey_pad",
        }
    return detect_black_stand(image)


def viewer_status(base_url: str) -> dict:
    with DIRECT_OPENER.open(f"{base_url}/status.json", timeout=2.0) as response:
        return json.loads(response.read().decode("utf-8"))


def fresh_main_frame(
    base_url: str,
    stream_name: str,
    previous_sequence: int,
    maximum_age_s: float,
) -> tuple[np.ndarray, int, float]:
    deadline = time.monotonic() + 4.0
    last_problem = "no viewer status"
    while time.monotonic() < deadline:
        status = viewer_status(base_url)
        sequence = int(status.get("sequence", {}).get(stream_name, 0))
        age = float(status.get("frame_age_s", {}).get(stream_name, float("inf")))
        healthy = bool(status.get("healthy", {}).get(stream_name))
        if healthy and sequence > previous_sequence and age <= maximum_age_s:
            capture = cv2.VideoCapture(f"{base_url}/{stream_name}.mjpg")
            ok, image = capture.read()
            capture.release()
            if ok and image is not None:
                return image, sequence, age
            last_problem = "MJPEG frame read failed"
        else:
            last_problem = (
                f"healthy={healthy}, sequence={sequence}, previous={previous_sequence}, age={age}"
            )
        time.sleep(0.12)
    raise RuntimeError(f"ZED main frame did not advance: {last_problem}")


def run_base_controller(arguments: list[str], timeout_s: float) -> dict:
    source = BASE_CONTROLLER.read_text(encoding="utf-8")
    remote = (
        f"{BASE_ENV}; timeout --signal=INT --kill-after=3 {timeout_s + 5:.1f} "
        "python3 - " + " ".join(arguments)
    )
    completed = subprocess.run(
        [
            "ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
            "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=3",
            BASE_HOST, remote,
        ],
        input=source,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout_s + 20.0,
        check=False,
    )
    if completed.returncode:
        raise RuntimeError(
            f"base controller failed ({completed.returncode}): {completed.stdout[-2000:]}"
        )
    report = _extract_last_json_object(completed.stdout)
    if report.get("status") not in {
        "success", "awaiting_distance_confirmation",
    }:
        raise RuntimeError(f"base controller did not report success: {report}")
    return report


def load_config(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if value.get("schema_version") != 1:
        raise ValueError("unsupported wall-dock config schema")
    if abs(float(value["rotation_ccw_deg"]) - 90.0) > 1e-9:
        raise ValueError("this standalone stage requires exactly 90 deg CCW")
    if abs(float(value["rear_wall_clearance_m"]) - 0.22) > 1e-9:
        raise ValueError("this standalone stage requires 0.22 m rear clearance")
    if value.get("wall_distance_translation_authorized") is not False:
        raise ValueError("wall-distance translation must remain unauthorized")
    if not 0.008 <= float(value["right_search_step_m"]) <= 0.08:
        raise ValueError("right search step must be in [0.008, 0.08] m")
    if not 0.0 < float(value["maximum_right_search_m"]) <= 3.2:
        raise ValueError("maximum right search must be in (0, 3.2] m")
    return value


def next_right_search_step(
    requested_m: float, maximum_m: float, nominal_step_m: float
) -> float | None:
    remaining = float(maximum_m) - float(requested_m)
    if remaining < 0.008 - 1e-9:
        return None
    return min(float(nominal_step_m), remaining)


def execute(config: dict, record_path: Path) -> int:
    report = {
        "schema_version": 1,
        "status": "running",
        "started_at_unix_s": time.time(),
        "integrated_into_full_pipeline": False,
        "uploaded": False,
        "completed_phases": [],
        "right_search_steps": [],
        "maximum_right_search_m": config["maximum_right_search_m"],
        "active_phase": "rotate_ccw_90",
    }
    atomic_write_json(record_path, report)
    try:
        rotation = run_base_controller([
            "--mode", "rotate",
            "--ccw-deg", f"{float(config['rotation_ccw_deg']):.3f}",
            "--yaw-speed-rps", f"{float(config['rotation_speed_rad_s']):.3f}",
            "--timeout-s", "70",
        ], 75.0)
        report["rotation"] = rotation
        report["completed_phases"].append("rotate_ccw_90")
        report["active_phase"] = "rear_wall_heading_parallel"
        atomic_write_json(record_path, report)

        wall = run_base_controller([
            "--mode", "wall-align",
            "--wall-clearance-m", f"{float(config['rear_wall_clearance_m']):.3f}",
            "--speed-mps", f"{float(config['wall_translation_speed_m_s']):.3f}",
            "--yaw-speed-rps", "0.080",
            "--timeout-s", "120",
            "--orientation-only",
        ], 125.0)
        report["rear_wall"] = wall
        report["completed_phases"].append("rear_wall_heading_parallel")
        report["active_phase"] = "awaiting_wall_distance_confirmation"
        report["status"] = "awaiting_wall_distance_confirmation"
        report["distance_intent"] = wall["distance_intent"]
        report["right_search_deferred"] = True
        atomic_write_json(record_path, report)
        return 0
    except KeyboardInterrupt:
        report["status"] = "interrupted"
        report["error"] = "operator interrupt"
        return 130
    except Exception as exc:
        report["status"] = "blocked"
        report["error"] = f"{type(exc).__name__}: {exc}"
        return 2
    finally:
        report["finished_at_unix_s"] = time.time()
        atomic_write_json(record_path, report)
        print(json.dumps(report, indent=2), flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--record", type=Path, default=DEFAULT_RECORD)
    args = parser.parse_args()
    config = load_config(args.config)
    if not args.execute:
        print(json.dumps({
            "status": "dry_run",
            "physical_motion_commanded": False,
            "integrated_into_full_pipeline": False,
            "ordered_phases": [
                "rotate_ccw_90",
                "rear_wall_heading_parallel",
                "report_forward_or_backward_intent_without_translation",
            ],
            "deferred_until_operator_confirmation": [
                "wall_distance_translation",
                "right_search_for_black_stand_up_to_3_2m",
            ],
            "config": config,
        }, indent=2))
        return 0

    lock = None
    try:
        lock = acquire_motion_lock()
        return execute(config, args.record)
    except MissionAlreadyRunning as exc:
        print(json.dumps({"status": "blocked", "error": str(exc)}, indent=2))
        return 73
    finally:
        release_motion_lock(lock)


if __name__ == "__main__":
    raise SystemExit(main())
