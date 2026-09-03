#!/usr/bin/env python3
"""Main-camera search guidance and wrist-camera closed-loop lateral alignment."""

from __future__ import annotations

import argparse
import json
import subprocess
import time
import cv2

from alignment_detector import (
    detect_main_hint,
    detect_occluded_grey_pad,
    detect_target,
    horizontal_decision,
    wrist_vertical_robot_decision,
)

VIEWER = "http://127.0.0.1:18081"
MOVE_SCRIPT = "/home/aup/tmr-mobile-manipulation/base/scripts/12_translate_right_odom_only.py"
ROS_SETUP = "source /home/aup/tmr_env.sh"
BASE_HOST = "tmr-user@172.16.0.50"
BASE_ENV = (
    "source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash; "
    "export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp "
    "CYCLONEDDS_URI=file:///home/tmr-user/cyclonedds.xml"
)


def frame(name: str):
    capture = cv2.VideoCapture(f"{VIEWER}/{name}.mjpg")
    ok, image = capture.read()
    capture.release()
    if not ok:
        raise RuntimeError(f"{name} camera unavailable")
    return image


def radar_ready() -> bool:
    command = (
        f"{BASE_ENV}; "
        "timeout 2 ros2 topic echo /lidar_front/scan --once >/dev/null 2>&1 && "
        "timeout 2 ros2 topic echo /lidar_rear/scan --once >/dev/null 2>&1"
    )
    return subprocess.run(["ssh", BASE_HOST, command], check=False).returncode == 0


def move_right(distance_m: float, allow_odom_only: bool = False) -> None:
    if not allow_odom_only and not radar_ready():
        raise RuntimeError("dual-radar safety gate unavailable; zero motion commanded")
    command = (
        f"{BASE_ENV}; cd ~/tmr_cycle; python3 scripts/12_translate_right_odom_only.py "
        f"--distance-m {distance_m:.5f} "
        "--speed-mps 0.025 --timeout-s 8"
    )
    subprocess.run(["ssh", BASE_HOST, command], check=True)


def observe() -> dict:
    main, wrist = frame("main"), frame("left")
    wrist_target = detect_target(wrist) or detect_occluded_grey_pad(wrist)
    main_target = detect_target(main) or detect_main_hint(main)
    selected = wrist_target if wrist_target else main_target
    source = "wrist" if wrist_target else "main"
    decision = (
        wrist_vertical_robot_decision(wrist_target, wrist.shape[0])
        if wrist_target
        else horizontal_decision(main_target, main.shape[1])
    )
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
        if not args.execute:
            print(json.dumps({"status": "dry_run", "history": history}, indent=2))
            return 0
        if state["decision"] == "not_visible":
            raise RuntimeError("target absent from both cameras; search direction is ambiguous")
        move_right(
            args.step_m if state["decision"] == "move_right" else -args.step_m,
            allow_odom_only=args.allow_odom_only,
        )
        time.sleep(0.5)
    raise RuntimeError("alignment step limit reached")


if __name__ == "__main__":
    raise SystemExit(main())
