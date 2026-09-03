#!/usr/bin/env python3
"""Retract/down link8; after halfway tilt down and open while finishing."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import rclpy
from control_msgs.action import GripperCommand

from execute_thermal_pad_grasp import ThermalPadExecutor
from set_stage1_start_from_current import wait_motion_inputs
from thermal_pad_ik import DEFAULT_CONFIG, ROOT, pose_values
from thermal_pad_sequence import tilt_axis_toward


DEFAULT_RECORD = ROOT / "config" / "latest_stage5_release.json"


def release_target(position, backward_m: float, down_m: float) -> np.ndarray:
    return np.asarray(position, dtype=float) + np.array(
        [-float(backward_m), 0.0, -float(down_m)]
    )


class OrderedRelease(ThermalPadExecutor):
    def begin_opening(self, opened_position: float):
        goal = GripperCommand.Goal()
        goal.command.position = float(opened_position)
        goal.command.max_effort = 1.0
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            raise RuntimeError("halfway gripper-open goal rejected")
        return handle

    def finish_opening(self, handle) -> dict:
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=35.0)
        wrapped = result_future.result() if result_future.done() else None
        if wrapped is None:
            self.cancel(handle)
            raise RuntimeError("halfway gripper-open goal timed out")
        result = wrapped.result
        report = {
            "label": "open_from_halfway_until_arm_complete",
            "action_status": int(wrapped.status),
            "position": float(result.position),
            "effort": float(result.effort),
            "stalled": bool(result.stalled),
            "reached_goal": bool(result.reached_goal),
        }
        if not report["reached_goal"]:
            raise RuntimeError("left gripper did not reach the fully open position")
        return report

    def retract_tilt_and_open(
        self,
        first_half: list[dict],
        second_half: list[dict],
        speed: float,
        opened_position: float,
    ) -> tuple[list[dict], dict]:
        """At 50% retreat, begin downward tilt and asynchronous gripper opening."""
        motions = []
        plan = first_half + second_half
        for index, waypoint in enumerate(first_half, 1):
            motions.append(self.move_ptp(
                waypoint["joint_positions_rad"],
                f"retract_and_down_{index}_of_{len(plan)}",
                speed,
            ))
        self.motion_gate()
        gripper_handle = self.begin_opening(opened_position)
        for index, waypoint in enumerate(second_half, len(first_half) + 1):
            motions.append(self.move_ptp(
                waypoint["joint_positions_rad"],
                f"retract_tilt_and_open_{index}_of_{len(plan)}",
                speed,
            ))
        self.motion_gate()
        gripper = self.finish_opening(gripper_handle)
        return motions, gripper


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--record", type=Path, default=DEFAULT_RECORD)
    parser.add_argument("--backward-m", type=float, default=0.11)
    parser.add_argument("--down-m", type=float, default=0.01)
    parser.add_argument("--tilt-down-deg", type=float, default=8.0)
    parser.add_argument("--speed-rad-s", type=float, default=0.012)
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required for physical motion")
    if not 0.10 <= args.backward_m <= 0.12:
        parser.error("--backward-m must be in [0.10, 0.12]")
    if not 0.0 < args.down_m <= 0.01:
        parser.error("--down-m must be in (0, 0.01]")
    if not 1.0 <= args.tilt_down_deg <= 12.0:
        parser.error("--tilt-down-deg must be in [1, 12]")

    config = json.loads(args.config.read_text(encoding="utf-8"))
    rclpy.init()
    node = OrderedRelease(config)
    node.isolated_base_zero_locked = True
    report = {
        "status": "starting",
        "requested_backward_m": args.backward_m,
        "requested_down_m": args.down_m,
        "gripper_motion": "begin_opening_at_halfway; fully_open_at_completion",
        "tilt_begins_at_retreat_fraction": 0.5,
        "tilt_down_deg": args.tilt_down_deg,
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
        midpoint = release_target(
            start_position, args.backward_m * 0.5, args.down_m * 0.5
        )
        motion = config["motion_sequence"]
        tilted_orientation = tilt_axis_toward(
            orientation,
            motion["link8_gripper_approach_axis_local_xyz"],
            -np.asarray(motion["ground_up_axis_xyz"], dtype=float),
            args.tilt_down_deg,
        ).tolist()
        first_half, seed = node.solve_pose_segment(
            "release_first_half_level",
            np.asarray(start_position, dtype=float),
            midpoint,
            orientation,
            orientation,
            node.joints,
        )
        second_half, _ = node.solve_pose_segment(
            "release_second_half_tilt_and_open",
            midpoint,
            target,
            orientation,
            tilted_orientation,
            seed,
        )
        report["before"] = {
            "joint_positions_rad": list(node.joints),
            "link8_base_position_m": start_position,
            "link8_base_orientation_xyzw": orientation,
        }
        report["target_link8_base_position_m"] = target.tolist()
        report["halfway_link8_base_position_m"] = midpoint.tolist()
        report["target_link8_base_orientation_xyzw"] = tilted_orientation
        opened = float(config["empty_cycle"]["open_position"])
        report["motions"], final_gripper = node.retract_tilt_and_open(
            first_half, second_half, args.speed_rad_s, opened
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
