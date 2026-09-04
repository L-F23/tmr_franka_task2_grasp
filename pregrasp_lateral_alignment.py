#!/usr/bin/env python3
"""Require grasp-pose lateral alignment before the left gripper may close."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy

from base_motion import guarded_move_right
from alignment_detector import detect_occluded_grey_pad, detect_target
from colored_pad_detector import detect_colored_pads
from execute_thermal_pad_grasp import ThermalPadExecutor
from thermal_pad_ik import DEFAULT_CONFIG, ROOT


VIEWER = "http://127.0.0.1:18081"
DEFAULT_ALIGNMENT_CONFIG = ROOT / "config" / "pregrasp_lateral_alignment.json"
DEFAULT_WRIST_MAPPING = ROOT / "config" / "wrist_lateral_mapping.json"
DEFAULT_RECORD = ROOT / "config" / "latest_pregrasp_lateral_alignment.json"


def frame(name: str) -> np.ndarray:
    capture = cv2.VideoCapture(f"{VIEWER}/{name}.mjpg")
    ok, image = capture.read()
    capture.release()
    if not ok or image is None:
        raise RuntimeError(f"{name} camera unavailable")
    return image


def wrist_template_match(image: np.ndarray, config: dict) -> dict:
    reference = cv2.imread(str(ROOT / config["wrist_reference_image"]))
    if reference is None:
        raise RuntimeError("wrist alignment reference image is unavailable")
    x, y, width, height = map(int, config["wrist_template_bbox_xywh"])
    template = cv2.cvtColor(reference[y:y + height, x:x + width], cv2.COLOR_BGR2GRAY)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    response = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
    _, confidence, _, location = cv2.minMaxLoc(response)
    return {
        "confidence": float(confidence),
        "top_left_px": [int(location[0]), int(location[1])],
        "template_size_px": [width, height],
    }


def wrist_decision(match_top_y: float, reference_top_y: float, deadband_px: float) -> str:
    error = float(match_top_y) - float(reference_top_y)
    if abs(error) <= float(deadband_px):
        return "aligned"
    # Operator-verified grasp-pose mapping: pad above gripper -> base right;
    # pad below gripper -> base left.
    return "move_right" if error < 0.0 else "move_left"


def mapped_correction_m(center_y_error_px: float, mapping_config: dict) -> float:
    coefficients = mapping_config["mapping"]["right_correction_m"]
    error = float(center_y_error_px)
    return (
        float(coefficients["linear_m_per_px"]) * error
        + float(coefficients["quadratic_m_per_px2"]) * error * error
    )


def mapped_wrist_state(
    image: np.ndarray, mapping_config: dict, deadband_px: float
) -> dict:
    reference = cv2.imread(str(ROOT / mapping_config["reference_image"]))
    if reference is None:
        raise RuntimeError("calibrated wrist mapping reference image is unavailable")
    x, y, width, height = map(int, mapping_config["reference_bbox_xywh"])
    template = cv2.cvtColor(reference[y:y + height, x:x + width], cv2.COLOR_BGR2GRAY)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    response = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
    _, confidence, _, location = cv2.minMaxLoc(response)
    if confidence < float(mapping_config["minimum_template_confidence"]):
        raise RuntimeError(f"wrist mapping confidence too low: {confidence:.3f}")
    center_y = float(location[1] + height / 2.0)
    error_y = center_y - float(mapping_config["reference_target_center_y_px"])
    lower, upper = map(float, mapping_config["valid_center_y_error_px"])
    inside_calibrated_range = lower <= error_y <= upper
    correction = mapped_correction_m(error_y, mapping_config)
    decision = "aligned" if abs(error_y) <= deadband_px else (
        "move_right" if correction > 0.0 else "move_left"
    )
    return {
        "source": "left_wrist_calibrated_mapping",
        "decision": decision,
        "target_center_y_px": center_y,
        "target_center_y_error_px": error_y,
        "recommended_right_correction_m": correction,
        "template_confidence": float(confidence),
        "inside_calibrated_range": inside_calibrated_range,
        "mapping_use": (
            "calibrated_magnitude_and_direction" if inside_calibrated_range
            else "operator_verified_direction_with_step_clamp"
        ),
    }


def main_guidance(image: np.ndarray, reference_red_x: float, deadband_px: float) -> dict:
    red = [item for item in detect_colored_pads(image) if item.color == "red"]
    if len(red) != 1:
        raise RuntimeError(
            f"wrist target outside view and main camera has {len(red)} reliable red pads"
        )
    red_x = float(red[0].center_px[0])
    error = red_x - float(reference_red_x)
    if abs(error) <= float(deadband_px):
        raise RuntimeError(
            "main camera is at the reference station but wrist target is not visible"
        )
    # Moving the base right shifts the fixed table layout left in the main view.
    return {
        "decision": "move_right" if error > 0.0 else "move_left",
        "red_center_x_px": red_x,
        "red_reference_x_px": float(reference_red_x),
        "red_error_x_px": error,
    }


def open_gripper(robot_config: dict) -> dict:
    rclpy.init()
    node = ThermalPadExecutor(robot_config)
    node.isolated_base_zero_locked = True
    try:
        if not node.gripper_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("left gripper action unavailable")
        return node.command_gripper(
            float(robot_config["empty_cycle"]["open_position"]),
            "open_before_mandatory_grasp_alignment",
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--config", type=Path, default=DEFAULT_ALIGNMENT_CONFIG)
    parser.add_argument("--wrist-mapping", type=Path, default=DEFAULT_WRIST_MAPPING)
    parser.add_argument("--robot-config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--record", type=Path, default=DEFAULT_RECORD)
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required for physical alignment")

    config = json.loads(args.config.read_text(encoding="utf-8"))
    mapping_config = json.loads(args.wrist_mapping.read_text(encoding="utf-8"))
    robot_config = json.loads(args.robot_config.read_text(encoding="utf-8"))
    report = {
        "status": "blocked",
        "semantics": "calibrated wrist-Y-to-base-right correction; main fallback",
        "wrist_gate": "operator-confirmed template and measured odometry mapping",
        "wrist_mapping": str(args.wrist_mapping),
        "base_steps": [],
        "history": [],
        "right_arm_commanded": False,
        "spine_commanded": False,
    }
    code = 2
    try:
        report["gripper_open"] = open_gripper(robot_config)
        consecutive = 0
        for _ in range(int(config["maximum_steps"])):
            wrist = frame("left")
            try:
                state = mapped_wrist_state(
                    wrist, mapping_config, config["wrist_mapping_deadband_px"]
                )
            except RuntimeError as wrist_error:
                state = {
                    "source": "main",
                    "wrist_structured_target_visible": False,
                    "wrist_mapping_error": str(wrist_error),
                }
                state.update(main_guidance(
                    frame("main"),
                    config["main_reference_red_x_px"],
                    config["main_deadband_px"],
                ))
            report["history"].append(state)
            print(json.dumps(state), flush=True)
            if state["decision"] == "aligned":
                consecutive += 1
                if consecutive >= int(config["required_consecutive_aligned_frames"]):
                    report["status"] = "pregrasp_lateral_alignment_confirmed"
                    report["aligned_at_unix_s"] = time.time()
                    report["alignment_source"] = "left_wrist"
                    code = 0
                    break
                time.sleep(0.25)
                continue
            consecutive = 0
            if state["source"] == "left_wrist_calibrated_mapping":
                recommended = float(state["recommended_right_correction_m"])
                signed_step = np.sign(recommended) * min(
                    float(config["maximum_mapped_step_m"]),
                    max(float(config["minimum_mapped_step_m"]), abs(recommended)),
                )
            else:
                signed_step = float(config["step_m"])
                if state["decision"] == "move_left":
                    signed_step = -signed_step
            report["base_steps"].append(guarded_move_right(
                signed_step,
                speed_mps=float(config["speed_mps"]),
                timeout_s=15.0,
            ))
            time.sleep(0.4)
        else:
            raise RuntimeError("mandatory pregrasp lateral alignment step limit reached")
    except Exception as exc:
        report["error"] = str(exc)
    finally:
        args.record.parent.mkdir(parents=True, exist_ok=True)
        args.record.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2), flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
