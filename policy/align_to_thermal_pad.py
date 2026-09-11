#!/usr/bin/env python3
"""Main-camera search guidance and wrist-camera closed-loop lateral alignment."""

from __future__ import annotations

import argparse
import json
import time
import cv2

from base_motion import guarded_move_right
from alignment_detector import (
    Target,
    detect_main_hint,
    detect_occluded_grey_pad,
    detect_target,
    horizontal_decision,
    wrist_vertical_robot_decision,
)

VIEWER = "http://localhost:18081"
def frame(name: str):
    capture = cv2.VideoCapture(f"{VIEWER}/{name}.mjpg")
    ok, image = capture.read()
    capture.release()
    if not ok:
        raise RuntimeError(f"{name} camera unavailable")
    return image


def main_table_target(image) -> Target | None:
    """Reject wall/robot candidates outside the calibrated table-height band."""
    target = detect_target(image) or detect_main_hint(image)
    if target is None:
        return None
    height = image.shape[0]
    if not 0.30 * height <= target.center[1] <= 0.88 * height:
        return None
    return target


def move_right(distance_m: float, allow_odom_only: bool = False) -> None:
    # The bundled mover is already odometry closed-loop.  Keep the legacy flag
    # as a CLI-compatible alias without depending on a remote Task 3 script.
    _ = allow_odom_only
    guarded_move_right(distance_m, speed_mps=0.025, timeout_s=15.0)


def observe() -> dict:
    main, wrist = frame("main"), frame("left")
    wrist_target = detect_target(wrist) or detect_occluded_grey_pad(wrist)
    main_target = main_table_target(main)
    selected = wrist_target if wrist_target else main_target
    source = "wrist" if wrist_target else "main"
    decision = (
        wrist_vertical_robot_decision(wrist_target, wrist.shape[0])
        if wrist_target
        else horizontal_decision(main_target, main.shape[1])
    )
    # At the stowed initial arm pose the wrist may not see the table at all.
    # Main centering completes only this coarse base stage; the mandatory
    # pregrasp wrist gate later rechecks lateral alignment before closing.
    if not wrist_target and decision == "centered":
        decision = "main_centered"
    return {
        "source": source,
        "decision": decision,
        "wrist_visible": wrist_target is not None,
        "main_visible": main_target is not None,
        "center": None if selected is None else list(selected.center),
        "confidence": None if selected is None else selected.confidence,
        "controlled_image_axis": "y" if wrist_target else "x",
        "image_size": list((wrist if wrist_target else main).shape[1::-1]),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--maximum-steps", type=int, default=12)
    parser.add_argument("--step-m", type=float, default=0.02)
    parser.add_argument("--allow-odom-only", action="store_true")
    args = parser.parse_args()
    history = []
    for _ in range(args.maximum_steps):
        state = observe()
        history.append(state)
        print(json.dumps(state), flush=True)
        if state["decision"] == "centered":
            print(json.dumps({"status": "centered", "history": history}, indent=2))
            return 0
        if state["decision"] == "main_centered":
            print(json.dumps({
                "status": "main_centered_wrist_deferred_to_mandatory_pregrasp_gate",
                "history": history,
            }, indent=2))
            return 0
        if not args.execute:
            print(json.dumps({"status": "dry_run", "history": history}, indent=2))
            return 0
        if state["decision"] == "not_visible":
            raise RuntimeError("target absent from both cameras; search direction is ambiguous")
        decision = state["decision"]
        move_right(
            args.step_m if decision == "move_right" else -args.step_m,
            allow_odom_only=args.allow_odom_only,
        )
        time.sleep(0.5)
    raise RuntimeError("alignment step limit reached")


if __name__ == "__main__":
    raise SystemExit(main())
