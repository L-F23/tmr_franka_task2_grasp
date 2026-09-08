#!/usr/bin/env python3
"""Reverse the recorded final stage-5 extra-tilt segment with gripper held."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import rclpy

from set_stage1_start_from_current import wait_motion_inputs
from stage5_release_diagonal import OrderedRelease
from thermal_pad_ik import DEFAULT_CONFIG, ROOT, pose_values, quaternion_angle_deg


DEFAULT_STAGE5_RECORD = ROOT / "config" / "latest_stage5_release.json"
DEFAULT_RECORD = ROOT / "config" / "latest_stage5_step_back_final_tilt.json"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--stage5-record", type=Path, default=DEFAULT_STAGE5_RECORD)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--record", type=Path, default=DEFAULT_RECORD)
    parser.add_argument("--speed-rad-s", type=float, default=0.04)
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required")

    stage5 = json.loads(args.stage5_record.read_text(encoding="utf-8"))
    config = json.loads(args.config.read_text(encoding="utf-8"))
    expected_position = np.asarray(
        stage5["final_combined_target_link8_base_position_m"], dtype=float
    )
    expected_orientation = stage5["final_combined_target_link8_orientation_xyzw"]
    target_position = np.asarray(
        stage5["target_link8_base_position_m"], dtype=float
    )
    target_orientation = stage5["target_link8_base_orientation_xyzw"]

    rclpy.init()
    node = OrderedRelease(config)
    node.fast_execution = True
    node.isolated_base_zero_locked = True
    report = {
        "status": "starting",
        "semantics": "reverse recorded final extra-tilt segment",
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
        current_pose = node.fk(node.joints)
        current_position, current_orientation = pose_values(current_pose)
        position_error = float(np.linalg.norm(
            np.asarray(current_position) - expected_position
        ))
        orientation_error = quaternion_angle_deg(
            current_orientation, expected_orientation
        )
        if position_error > 0.015 or orientation_error > 5.0:
            raise RuntimeError(
                "current pose is not the recorded final-tilt endpoint: "
                f"position={position_error:.4f}m, "
                f"orientation={orientation_error:.2f}deg"
            )
        plan, _ = node.solve_pose_segment(
            "reverse_stage5_final_extra_tilt",
            np.asarray(current_position), target_position,
            current_orientation, target_orientation, node.joints,
        )
        report["before"] = {
            "link8_base_position_m": current_position,
            "link8_base_orientation_xyzw": current_orientation,
            "recorded_endpoint_position_error_m": position_error,
            "recorded_endpoint_orientation_error_deg": orientation_error,
        }
        report["target"] = {
            "link8_base_position_m": target_position.tolist(),
            "link8_base_orientation_xyzw": target_orientation,
        }
        report["motions"] = []
        for index, waypoint in enumerate(plan, 1):
            report["motions"].append(node.move_ptp(
                waypoint["joint_positions_rad"],
                f"reverse_final_tilt_{index}_of_{len(plan)}",
                args.speed_rad_s,
            ))
        after_pose = node.fk(node.joints)
        after_position, after_orientation = pose_values(after_pose)
        report["after"] = {
            "link8_base_position_m": after_position,
            "link8_base_orientation_xyzw": after_orientation,
            "target_position_error_m": float(np.linalg.norm(
                np.asarray(after_position) - target_position
            )),
            "target_orientation_error_deg": quaternion_angle_deg(
                after_orientation, target_orientation
            ),
            "active_errors": node.active_errors(),
        }
        report["status"] = "final_extra_tilt_reversed_gripper_still_held"
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
