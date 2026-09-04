#!/usr/bin/env python3
"""Resume the two recorded stage-5 terminal segments after a returned probe."""

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
DEFAULT_RECORD = ROOT / "config" / "latest_stage5_resume_after_terminal_calibration.json"


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

    saved = json.loads(args.stage5_record.read_text(encoding="utf-8"))
    config = json.loads(args.config.read_text(encoding="utf-8"))
    expected_position = np.asarray(saved["target_link8_base_position_m"], dtype=float)
    expected_orientation = saved["target_link8_base_orientation_xyzw"]
    final_position = np.asarray(
        saved["final_combined_target_link8_base_position_m"], dtype=float
    )
    final_orientation = saved["final_combined_target_link8_orientation_xyzw"]
    follow_position = np.asarray(
        saved["follow_through_target_link8_base_position_m"], dtype=float
    )
    follow_orientation = saved["follow_through_target_link8_orientation_xyzw"]

    rclpy.init()
    node = OrderedRelease(config)
    node.fast_execution = True
    node.isolated_base_zero_locked = True
    report = {
        "status": "starting",
        "gripper_commanded": False,
        "base_commanded": False,
        "right_arm_commanded": False,
        "spine_commanded": False,
        "visual_residual_applied": False,
        "visual_residual_rejected_reason": (
            "red contour changed under pad occlusion; FK return was sub-millimetre"
        ),
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
        if position_error > 0.012 or orientation_error > 3.0:
            raise RuntimeError(
                "current pose is not the calibrated stage-5 resume point: "
                f"position={position_error:.4f}m, orientation={orientation_error:.2f}deg"
            )
        report["resume_gate"] = {
            "position_error_m": position_error,
            "orientation_error_deg": orientation_error,
        }
        report["motions"] = []
        for label, target_position, target_orientation in (
            ("final_extra_tilt", final_position, final_orientation),
            ("follow_through", follow_position, follow_orientation),
        ):
            pose = node.fk(node.joints)
            position, orientation = pose_values(pose)
            plan, _ = node.solve_pose_segment(
                f"resume_{label}",
                np.asarray(position), target_position,
                orientation, target_orientation, node.joints,
            )
            for index, waypoint in enumerate(plan, 1):
                report["motions"].append(node.move_ptp(
                    waypoint["joint_positions_rad"],
                    f"resume_{label}_{index}_of_{len(plan)}",
                    args.speed_rad_s,
                ))
        after_pose = node.fk(node.joints)
        after_position, after_orientation = pose_values(after_pose)
        report["after"] = {
            "link8_base_position_m": after_position,
            "link8_base_orientation_xyzw": after_orientation,
            "target_position_error_m": float(np.linalg.norm(
                np.asarray(after_position) - follow_position
            )),
            "target_orientation_error_deg": quaternion_angle_deg(
                after_orientation, follow_orientation
            ),
            "active_errors": node.active_errors(),
        }
        report["status"] = "stage5_terminal_segments_resumed_gripper_still_held"
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
