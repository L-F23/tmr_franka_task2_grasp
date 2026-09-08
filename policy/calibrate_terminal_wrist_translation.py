#!/usr/bin/env python3
"""Probe 10 mm robot-left at release pose and calibrate wrist red-feature motion."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy

from execute_thermal_pad_grasp import ThermalPadExecutor
from set_stage1_start_from_current import wait_motion_inputs
from thermal_pad_ik import DEFAULT_CONFIG, ROOT, pose_values, quaternion_angle_deg


VIEWER = "http://127.0.0.1:18081"
DEFAULT_CAPTURE_DIR = ROOT / "captures" / "terminal_wrist_mapping_20260904"
DEFAULT_CONFIG_OUTPUT = ROOT / "config" / "terminal_wrist_translation_mapping.json"
DEFAULT_RECORD = ROOT / "config" / "latest_terminal_wrist_translation_calibration.json"


def frame() -> np.ndarray:
    capture = cv2.VideoCapture(f"{VIEWER}/left.mjpg")
    ok, image = capture.read()
    capture.release()
    if not ok or image is None:
        raise RuntimeError("left wrist camera unavailable")
    return image


def red_feature(image: np.ndarray) -> dict:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue, saturation, value = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    mask = np.uint8(
        (((hue <= 14) | (hue >= 170)) & (saturation >= 130) & (value >= 45))
    ) * 255
    height, width = mask.shape
    allowed = np.zeros_like(mask)
    allowed[int(.20 * height):int(.96 * height), int(.35 * width):] = 255
    mask &= allowed
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [contour for contour in contours if cv2.contourArea(contour) >= 300]
    if not contours:
        raise RuntimeError("stable red region not visible in left wrist")
    contour = max(contours, key=cv2.contourArea)
    moments = cv2.moments(contour)
    if moments["m00"] <= 0:
        raise RuntimeError("invalid red-region moments")
    x, y, box_width, box_height = cv2.boundingRect(contour)
    return {
        "center_px": [
            float(moments["m10"] / moments["m00"]),
            float(moments["m01"] / moments["m00"]),
        ],
        "bbox_xywh": [x, y, box_width, box_height],
        "area_px": float(cv2.contourArea(contour)),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--probe-left-m", type=float, default=0.01)
    parser.add_argument("--speed-rad-s", type=float, default=0.025)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--capture-dir", type=Path, default=DEFAULT_CAPTURE_DIR)
    parser.add_argument("--mapping-output", type=Path, default=DEFAULT_CONFIG_OUTPUT)
    parser.add_argument("--record", type=Path, default=DEFAULT_RECORD)
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required")
    if not 0.005 <= args.probe_left_m <= 0.015:
        parser.error("--probe-left-m must be in [0.005, 0.015]")

    config = json.loads(args.config.read_text(encoding="utf-8"))
    args.capture_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = ThermalPadExecutor(config)
    node.fast_execution = True
    node.isolated_base_zero_locked = True
    report = {
        "schema_version": 1,
        "status": "starting",
        "probe_axis": "base_positive_y_robot_left",
        "probe_left_m": args.probe_left_m,
        "gripper_commanded": False,
        "base_commanded": False,
        "right_arm_commanded": False,
        "spine_commanded": False,
    }
    code = 2
    try:
        if not node.ptp_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("left PTP action unavailable")
        wait_motion_inputs(node)
        errors = node.active_errors()
        if errors:
            raise RuntimeError("Franka errors: " + ",".join(errors))
        start_pose = node.fk(node.joints)
        start_position, start_orientation = pose_values(start_pose)
        image_0 = frame()
        feature_0 = red_feature(image_0)
        image_0_path = args.capture_dir / "calibration_point_left.jpg"
        cv2.imwrite(str(image_0_path), image_0)

        probe_position = np.asarray(start_position, dtype=float) + np.array([
            0.0, args.probe_left_m, 0.0
        ])
        plan, _ = node.solve_pose_segment(
            "terminal_wrist_left_probe",
            np.asarray(start_position), probe_position,
            start_orientation, start_orientation, node.joints,
        )
        report["probe_motions"] = []
        for index, waypoint in enumerate(plan, 1):
            report["probe_motions"].append(node.move_ptp(
                waypoint["joint_positions_rad"],
                f"terminal_probe_left_{index}_of_{len(plan)}",
                args.speed_rad_s,
            ))
        time.sleep(0.25)
        image_1 = frame()
        feature_1 = red_feature(image_1)
        image_1_path = args.capture_dir / "after_robot_left_10mm_left.jpg"
        cv2.imwrite(str(image_1_path), image_1)

        measured_probe_pose = node.fk(node.joints)
        measured_probe_position, _ = pose_values(measured_probe_pose)
        actual_probe = float(
            np.asarray(measured_probe_position)[1] - np.asarray(start_position)[1]
        )
        if actual_probe < 0.004:
            raise RuntimeError(f"left probe made insufficient progress: {actual_probe:.6f}m")

        return_plan, _ = node.solve_pose_segment(
            "terminal_wrist_return_to_calibration_point",
            np.asarray(measured_probe_position), np.asarray(start_position),
            start_orientation, start_orientation, node.joints,
        )
        report["return_motions"] = []
        for index, waypoint in enumerate(return_plan, 1):
            report["return_motions"].append(node.move_ptp(
                waypoint["joint_positions_rad"],
                f"terminal_probe_return_{index}_of_{len(return_plan)}",
                args.speed_rad_s,
            ))
        returned_pose = node.fk(node.joints)
        returned_position, returned_orientation = pose_values(returned_pose)

        delta_px = np.asarray(feature_1["center_px"]) - np.asarray(feature_0["center_px"])
        squared = float(np.dot(delta_px, delta_px))
        if squared < 4.0:
            raise RuntimeError(f"red feature moved too little for mapping: {delta_px.tolist()}")
        correction_vector = (-actual_probe * delta_px / squared).tolist()
        mapping = {
            "schema_version": 1,
            "fixed_pose": "stage5_main_retract_and_20deg_tilt_endpoint",
            "reference_image": str(image_0_path.relative_to(ROOT)),
            "probe_image": str(image_1_path.relative_to(ROOT)),
            "reference_red_feature": feature_0,
            "probe_red_feature": feature_1,
            "probe_axis": "base_positive_y_robot_left",
            "actual_probe_left_m": actual_probe,
            "red_center_delta_px_for_probe": delta_px.tolist(),
            "red_center_jacobian_px_per_left_m": (delta_px / actual_probe).tolist(),
            "return_to_reference_left_correction_m_per_pixel_xy": correction_vector,
            "usage": (
                "dot(observed_center-reference_center, correction_vector) gives "
                "the signed base-positive-Y tool correction to return to reference"
            ),
        }
        args.mapping_output.write_text(
            json.dumps(mapping, indent=2) + "\n", encoding="utf-8"
        )
        report.update({
            "status": "calibration_complete_returned_to_reference",
            "reference_image": mapping["reference_image"],
            "probe_image": mapping["probe_image"],
            "mapping": mapping,
            "return_position_error_m": float(np.linalg.norm(
                np.asarray(returned_position) - np.asarray(start_position)
            )),
            "return_orientation_error_deg": quaternion_angle_deg(
                returned_orientation, start_orientation
            ),
            "active_errors": node.active_errors(),
        })
        code = 0
    except Exception as exc:
        report["status"] = "blocked"
        report["error"] = str(exc)
    finally:
        args.record.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
