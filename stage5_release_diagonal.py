#!/usr/bin/env python3
"""Continuously retract and dump the tool, opening near 6 cm retreat."""

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
from thermal_pad_sequence import quaternion_matrix, slerp, tilt_axis_toward


DEFAULT_RECORD = ROOT / "config" / "latest_stage5_release.json"


def release_target(position, backward_m: float, down_m: float) -> np.ndarray:
    return np.asarray(position, dtype=float) + np.array(
        [-float(backward_m), 0.0, -float(down_m)]
    )


def front_loaded_descent_fraction(fraction: float, power: float = 2.0) -> float:
    """Ease-out descent: larger vertical increments first, smaller ones later."""
    value = float(np.clip(fraction, 0.0, 1.0))
    if power < 1.0:
        raise ValueError("descent profile power must be at least 1")
    return 1.0 - (1.0 - value) ** float(power)


def clearance_compensated_position(
    raw_link8_position, orientation, contact_local, minimum_contact_z: float,
) -> tuple[np.ndarray, float]:
    """Raise link8 only as needed so the finger-contact end clears the table."""
    position = np.asarray(raw_link8_position, dtype=float).copy()
    contact = position + quaternion_matrix(orientation) @ np.asarray(contact_local, dtype=float)
    compensation = max(0.0, float(minimum_contact_z) - float(contact[2]))
    position[2] += compensation
    return position, compensation


class OrderedRelease(ThermalPadExecutor):
    def begin_opening(self, opened_position: float):
        self.requested_open_position = float(opened_position)
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
        report["measured_open_position_accepted"] = (
            report["position"] <= float(getattr(self, "requested_open_position", 0.0)) + 0.02
        )
        if not (report["reached_goal"] or report["measured_open_position_accepted"]):
            raise RuntimeError("left gripper did not reach the fully open position")
        return report

    def retract_tilt_and_open(
        self, plan: list[dict], start_x: float, open_after_m: float,
        speed: float, opened_position: float, progress,
    ) -> dict:
        """Tilt from the first waypoint, open at 6 cm, then keep retracting."""
        gripper_handle = None
        for index, waypoint in enumerate(plan, 1):
            result = self.move_ptp(
                waypoint["joint_positions_rad"],
                f"retract_dump_{index}_of_{len(plan)}",
                speed,
            )
            retreat = float(start_x) - float(waypoint["position_m"][0])
            progress("motion", {**result, "retreat_m": retreat})
            if gripper_handle is None and retreat >= float(open_after_m) - 1e-6:
                self.motion_gate()
                gripper_handle = self.begin_opening(opened_position)
                progress("gripper_started", {"retreat_m": retreat})
        if gripper_handle is None:
            raise RuntimeError("release plan never reached the gripper-open distance")
        self.motion_gate()
        gripper = self.finish_opening(gripper_handle)
        progress("gripper_finished", gripper)
        return gripper


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--record", type=Path, default=DEFAULT_RECORD)
    parser.add_argument("--backward-m", type=float, default=0.11)
    parser.add_argument("--initial-down-m", type=float, default=0.008)
    parser.add_argument("--down-m", type=float, default=0.062)
    parser.add_argument("--tilt-down-deg", type=float, default=90.0)
    parser.add_argument("--open-after-m", type=float, default=0.06)
    parser.add_argument("--pre-open-contact-drop-m", type=float, default=0.015)
    parser.add_argument("--maximum-contact-drop-m", type=float, default=0.07)
    parser.add_argument("--speed-rad-s", type=float, default=0.05)
    parser.add_argument("--descent-ease-power", type=float, default=2.0)
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required for physical motion")
    if not 0.10 <= args.backward_m <= 0.12:
        parser.error("--backward-m must be in [0.10, 0.12]")
    if not 0.0 <= args.down_m <= 0.07:
        parser.error("--down-m must be in [0, 0.07]")
    if not 0.0 < args.initial_down_m <= 0.01:
        parser.error("--initial-down-m must be in (0, 0.01]")
    if not 1.0 <= args.tilt_down_deg <= 90.0:
        parser.error("--tilt-down-deg must be in [1, 90]")
    if not 0.04 <= args.open_after_m < args.backward_m:
        parser.error("--open-after-m must be in [0.04, backward-m)")
    if not args.initial_down_m <= args.pre_open_contact_drop_m <= 0.025:
        parser.error("--pre-open-contact-drop-m must be between initial-down and 0.025")
    if not args.pre_open_contact_drop_m <= args.maximum_contact_drop_m <= 0.08:
        parser.error("--maximum-contact-drop-m must be between pre-open drop and 0.08")
    if args.initial_down_m + args.down_m > args.maximum_contact_drop_m + 1e-9:
        parser.error("initial-down-m + down-m cannot exceed maximum-contact-drop-m")
    if not 1.0 < args.descent_ease_power <= 4.0:
        parser.error("--descent-ease-power must be in (1, 4]")

    config = json.loads(args.config.read_text(encoding="utf-8"))
    rclpy.init()
    node = OrderedRelease(config)
    node.fast_execution = True
    node.isolated_base_zero_locked = True
    report = {
        "status": "starting",
        "requested_backward_m": args.backward_m,
        "requested_initial_down_m": args.initial_down_m,
        "requested_down_m": args.down_m,
        "descent_profile": "front_loaded_ease_out",
        "descent_ease_power": args.descent_ease_power,
        "gripper_motion": "begin_opening_near_6cm; continue_retracting_while_opening",
        "open_after_retreat_m": args.open_after_m,
        "tilt_begins_at_retreat_fraction": 0.0,
        "tilt_down_deg": args.tilt_down_deg,
        "maximum_finger_contact_drop_m": args.maximum_contact_drop_m,
        "pre_open_finger_contact_drop_m": args.pre_open_contact_drop_m,
        "clearance_guard": "link8 z is raised as needed to keep the finger contact end above its start-relative floor",
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
        lowered_start = release_target(start_position, 0.0, args.initial_down_m)
        raw_target = release_target(lowered_start, args.backward_m, args.down_m)
        motion = config["motion_sequence"]
        tilted_orientation = tilt_axis_toward(
            orientation,
            motion["link8_gripper_approach_axis_local_xyz"],
            -np.asarray(motion["ground_up_axis_xyz"], dtype=float),
            args.tilt_down_deg,
        ).tolist()
        contact_local = np.asarray(config["grasp"]["link8_to_finger_contact_local_m"], dtype=float)
        start_contact = np.asarray(start_position) + quaternion_matrix(orientation) @ contact_local
        plan, seed = node.solve_pose_segment(
            "release_initial_down_8mm",
            np.asarray(start_position, dtype=float), lowered_start,
            orientation, orientation, node.joints,
        )
        previous_position = np.asarray(lowered_start, dtype=float)
        previous_orientation = np.asarray(orientation, dtype=float)
        clearance_compensations = []
        allowed_contact_drops = []
        descent_fractions = []
        open_fraction = args.open_after_m / args.backward_m
        # Ten milestones make translation and the 90-degree dump simultaneous.
        for index, fraction in enumerate(np.linspace(0.1, 1.0, 10), 1):
            descent_fraction = front_loaded_descent_fraction(
                float(fraction), args.descent_ease_power
            )
            raw_position = np.asarray(lowered_start, dtype=float) + np.array([
                -args.backward_m * float(fraction),
                0.0,
                -args.down_m * descent_fraction,
            ])
            milestone_orientation = slerp(orientation, tilted_orientation, float(fraction))
            if fraction <= open_fraction:
                allowed_drop = args.initial_down_m + (
                    args.pre_open_contact_drop_m - args.initial_down_m
                ) * float(fraction) / open_fraction
            else:
                allowed_drop = args.pre_open_contact_drop_m + (
                    args.maximum_contact_drop_m - args.pre_open_contact_drop_m
                ) * (float(fraction) - open_fraction) / (1.0 - open_fraction)
            minimum_contact_z = float(start_contact[2]) - allowed_drop
            milestone_position, compensation = clearance_compensated_position(
                raw_position, milestone_orientation, contact_local, minimum_contact_z
            )
            segment, seed = node.solve_pose_segment(
                f"release_dump_{index}_of_10",
                previous_position, milestone_position,
                previous_orientation, milestone_orientation, seed,
            )
            plan.extend(segment)
            clearance_compensations.append(float(compensation))
            allowed_contact_drops.append(float(allowed_drop))
            descent_fractions.append(float(descent_fraction))
            previous_position = milestone_position
            previous_orientation = milestone_orientation
        report["before"] = {
            "joint_positions_rad": list(node.joints),
            "link8_base_position_m": start_position,
            "link8_base_orientation_xyzw": orientation,
        }
        report["raw_target_link8_base_position_m"] = raw_target.tolist()
        report["initial_lowered_link8_base_position_m"] = lowered_start.tolist()
        report["target_link8_base_position_m"] = previous_position.tolist()
        report["target_link8_base_orientation_xyzw"] = tilted_orientation
        report["final_minimum_finger_contact_base_z_m"] = minimum_contact_z
        report["allowed_contact_drop_by_milestone_m"] = allowed_contact_drops
        report["descent_fraction_by_milestone"] = descent_fractions
        report["clearance_compensation_by_milestone_m"] = clearance_compensations
        report["planned_waypoint_count"] = len(plan)
        opened = float(config["empty_cycle"]["open_position"])
        def progress(kind, value):
            if kind == "motion":
                report["motions"].append(value)
            else:
                report[kind] = value
            args.record.parent.mkdir(parents=True, exist_ok=True)
            args.record.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

        final_gripper = node.retract_tilt_and_open(
            plan, float(start_position[0]), args.open_after_m,
            args.speed_rad_s, opened, progress,
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
        report["resume_requires_fresh_current_pose_replan"] = True
    finally:
        args.record.parent.mkdir(parents=True, exist_ok=True)
        args.record.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
