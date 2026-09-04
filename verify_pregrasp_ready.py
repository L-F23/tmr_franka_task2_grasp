#!/usr/bin/env python3
"""Read-only gate for starting the Task 2 cycle at the calibrated pregrasp pose."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy

from execute_thermal_pad_grasp import ThermalPadExecutor
from set_stage1_start_from_current import pose_record, wait_motion_inputs
from thermal_pad_ik import DEFAULT_CONFIG, quaternion_angle_deg


def pose_errors(measured: dict, reference: dict) -> dict:
    measured_joints = np.asarray(measured["joint_positions_rad"], dtype=float)
    expected_joints = np.asarray(reference["expected_joint_positions_rad"], dtype=float)
    measured_position = np.asarray(measured["link8_base_position_m"], dtype=float)
    expected_position = np.asarray(reference["expected_link8_base_position_m"], dtype=float)
    return {
        "maximum_joint_error_rad": float(np.max(np.abs(measured_joints - expected_joints))),
        "position_error_m": float(np.linalg.norm(measured_position - expected_position)),
        "orientation_error_deg": quaternion_angle_deg(
            measured["link8_base_orientation_xyzw"],
            reference["expected_link8_base_orientation_xyzw"],
        ),
    }


def validate_errors(errors: dict, reference: dict) -> None:
    limits = {
        "maximum_joint_error_rad": float(reference["maximum_joint_error_rad"]),
        "position_error_m": float(reference["maximum_position_error_m"]),
        "orientation_error_deg": float(reference["maximum_orientation_error_deg"]),
    }
    exceeded = [
        f"{name}={errors[name]:.6f}>{limit:.6f}"
        for name, limit in limits.items()
        if float(errors[name]) > limit
    ]
    if exceeded:
        raise RuntimeError("left arm is not at the calibrated pregrasp pose: " + ", ".join(exceeded))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--record", type=Path,
        default=DEFAULT_CONFIG.parent / "latest_pregrasp_ready_check.json",
    )
    args = parser.parse_args()

    config = json.loads(args.config.read_text(encoding="utf-8"))
    reference = config["pregrasp_start"]
    report = {
        "status": "checking",
        "checked_at_unix_s": time.time(),
        "physical_motion_commanded": False,
        "base_commanded": False,
        "right_arm_commanded": False,
        "spine_commanded": False,
    }
    code = 2
    rclpy.init()
    node = ThermalPadExecutor(config)
    node.isolated_base_zero_locked = True
    try:
        if not node.ptp_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("left PTP action unavailable")
        if not node.gripper_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("left gripper action unavailable")
        wait_motion_inputs(node)
        measured = pose_record(node)
        report["measured"] = measured
        if measured["active_errors"]:
            raise RuntimeError("Franka errors: " + ",".join(measured["active_errors"]))
        errors = pose_errors(measured, reference)
        report["errors"] = errors
        report["limits"] = {
            "maximum_joint_error_rad": reference["maximum_joint_error_rad"],
            "position_error_m": reference["maximum_position_error_m"],
            "orientation_error_deg": reference["maximum_orientation_error_deg"],
        }
        validate_errors(errors, reference)
        report["status"] = "pregrasp_ready"
        code = 0
    except Exception as exc:
        report["status"] = "blocked"
        report["error"] = str(exc)
    finally:
        args.record.parent.mkdir(parents=True, exist_ok=True)
        args.record.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
