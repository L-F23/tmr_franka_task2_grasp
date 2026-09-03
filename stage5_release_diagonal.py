#!/usr/bin/env python3
"""Retract/down the left link8 completely, then open the gripper."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import rclpy

from execute_thermal_pad_grasp import ThermalPadExecutor
from set_stage1_start_from_current import wait_motion_inputs
from thermal_pad_ik import DEFAULT_CONFIG, ROOT, pose_values


DEFAULT_RECORD = ROOT / "config" / "latest_stage5_release.json"


def release_target(position, backward_m: float, down_m: float) -> np.ndarray:
    return np.asarray(position, dtype=float) + np.array(
        [-float(backward_m), 0.0, -float(down_m)]
    )


class OrderedRelease(ThermalPadExecutor):
    def retract_then_open(
        self, plan: list[dict], speed: float, opened_position: float
    ) -> tuple[list[dict], dict]:
        """Verify every arm waypoint before issuing the only open command."""
        motions = []
        for index, waypoint in enumerate(plan, 1):
            motions.append(self.move_ptp(
                waypoint["joint_positions_rad"],
                f"retract_and_down_{index}_of_{len(plan)}",
                speed,
            ))
        self.motion_gate()
        gripper = self.command_gripper(
            opened_position, "open_after_arm_motion_complete"
        )
        if not gripper["reached_goal"]:
            raise RuntimeError("left gripper did not reach the fully open position")
        return motions, gripper


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--record", type=Path, default=DEFAULT_RECORD)
    parser.add_argument("--backward-m", type=float, default=0.11)
    parser.add_argument("--down-m", type=float, default=0.01)
    parser.add_argument("--speed-rad-s", type=float, default=0.012)
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required for physical motion")
    if not 0.10 <= args.backward_m <= 0.12:
        parser.error("--backward-m must be in [0.10, 0.12]")
    if not 0.0 < args.down_m <= 0.01:
        parser.error("--down-m must be in (0, 0.01]")

    config = json.loads(args.config.read_text(encoding="utf-8"))
    rclpy.init()
    node = OrderedRelease(config)
    node.isolated_base_zero_locked = True
    report = {
        "status": "starting",
        "requested_backward_m": args.backward_m,
        "requested_down_m": args.down_m,
        "gripper_motion": "open_only_after_arm_motion_complete",
        "base_commanded": False,
        "right_arm_commanded": False,
        "spine_commanded": False,
        "motions": [],
    }
    code = 2
    try:
        if not node.ptp_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("left PTP action unavailable")
        if not node.gripper_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("left gripper action unavailable")
        wait_motion_inputs(node)
        errors = node.active_errors()
        if errors:
            raise RuntimeError("Franka errors: " + ",".join(errors))
        start_pose = node.fk(node.joints)
        start_position, orientation = pose_values(start_pose)
        target = release_target(start_position, args.backward_m, args.down_m)
        plan, _ = node.solve_pose_segment(
            "synchronized_release_diagonal",
            np.asarray(start_position, dtype=float),
            target,
            orientation,
            orientation,
            node.joints,
        )
        report["before"] = {
            "joint_positions_rad": list(node.joints),
            "link8_base_position_m": start_position,
            "link8_base_orientation_xyzw": orientation,
        }
        report["target_link8_base_position_m"] = target.tolist()
        opened = float(config["empty_cycle"]["open_position"])
        report["motions"], final_gripper = node.retract_then_open(
            plan, args.speed_rad_s, opened
        )
        after_pose = node.fk(node.joints)
        after_position, after_orientation = pose_values(after_pose)
        report["after"] = {
            "joint_positions_rad": list(node.joints),
            "link8_base_position_m": after_position,
            "link8_base_orientation_xyzw": after_orientation,
            "gripper": final_gripper,
            "active_errors": node.active_errors(),
        }
        report["actual_delta_m"] = (
            np.asarray(after_position) - np.asarray(start_position)
        ).tolist()
        report["status"] = "released"
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
