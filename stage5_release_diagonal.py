#!/usr/bin/env python3
"""Retract/down the left link8 while opening the gripper in synchronized steps."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from franka_msgs.action import PTPMotion

from execute_thermal_pad_grasp import ThermalPadExecutor
from set_stage1_start_from_current import wait_motion_inputs
from thermal_pad_ik import DEFAULT_CONFIG, ROOT, pose_values


DEFAULT_RECORD = ROOT / "config" / "latest_stage5_release.json"


def release_target(position, backward_m: float, down_m: float) -> np.ndarray:
    return np.asarray(position, dtype=float) + np.array(
        [-float(backward_m), 0.0, -float(down_m)]
    )


class SynchronizedRelease(ThermalPadExecutor):
    def move_and_open(
        self, joints: list[float], gripper_position: float, label: str, speed: float
    ) -> dict:
        self.motion_gate()
        arm_goal = PTPMotion.Goal()
        arm_goal.goal_joint_configuration = list(map(float, joints))
        arm_goal.maximum_joint_velocities = [float(speed)] * 7
        arm_goal.goal_tolerance = 0.006
        gripper_goal = GripperCommand.Goal()
        gripper_goal.command.position = float(gripper_position)
        gripper_goal.command.max_effort = 1.0

        arm_send = self.ptp_client.send_goal_async(arm_goal)
        gripper_send = self.gripper_client.send_goal_async(gripper_goal)
        deadline = time.monotonic() + 8.0
        while (
            (not arm_send.done() or not gripper_send.done())
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.03)
        arm_handle = arm_send.result() if arm_send.done() else None
        gripper_handle = gripper_send.result() if gripper_send.done() else None
        if arm_handle is None or not arm_handle.accepted:
            raise RuntimeError(f"{label} arm goal rejected")
        if gripper_handle is None or not gripper_handle.accepted:
            self.cancel(arm_handle)
            raise RuntimeError(f"{label} gripper goal rejected")

        arm_result = arm_handle.get_result_async()
        gripper_result = gripper_handle.get_result_async()
        deadline = time.monotonic() + 35.0
        while (
            (not arm_result.done() or not gripper_result.done())
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.03)
            self.motion_gate(arm_handle)
        if not arm_result.done():
            self.cancel(arm_handle)
            raise RuntimeError(f"{label} arm timeout")
        if not gripper_result.done():
            self.cancel(gripper_handle)
            raise RuntimeError(f"{label} gripper timeout")
        arm_wrapped = arm_result.result()
        gripper_wrapped = gripper_result.result()
        if arm_wrapped is None or arm_wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"{label} arm failed with status "
                f"{getattr(arm_wrapped, 'status', None)}"
            )
        if gripper_wrapped is None or gripper_wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"{label} gripper failed with status "
                f"{getattr(gripper_wrapped, 'status', None)}"
            )
        settle = time.monotonic() + 0.20
        while time.monotonic() < settle:
            rclpy.spin_once(self, timeout_sec=0.03)
        endpoint_error = float(np.max(
            np.abs(np.asarray(self.joints, dtype=float) - np.asarray(joints, dtype=float))
        ))
        if endpoint_error > 0.012:
            raise RuntimeError(f"{label} joint endpoint error {endpoint_error:.6f} rad")
        grip = gripper_wrapped.result
        return {
            "label": label,
            "maximum_joint_error_rad": endpoint_error,
            "commanded_gripper_position": float(gripper_position),
            "measured_gripper_position": float(grip.position),
            "gripper_reached_goal": bool(grip.reached_goal),
            "gripper_stalled": bool(grip.stalled),
        }


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
    node = SynchronizedRelease(config)
    node.isolated_base_zero_locked = True
    report = {
        "status": "starting",
        "requested_backward_m": args.backward_m,
        "requested_down_m": args.down_m,
        "gripper_motion": "synchronized_linear_open",
        "base_commanded": False,
        "right_arm_commanded": False,
        "spine_commanded": False,
        "steps": [],
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
        closed = float(config["empty_cycle"]["closed_position"])
        opened = float(config["empty_cycle"]["open_position"])
        for index, waypoint in enumerate(plan, 1):
            fraction = index / len(plan)
            gripper_position = closed + fraction * (opened - closed)
            report["steps"].append(node.move_and_open(
                waypoint["joint_positions_rad"],
                gripper_position,
                f"release_{index}_of_{len(plan)}",
                args.speed_rad_s,
            ))
        final_gripper = node.command_gripper(opened, "confirm_fully_open")
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
