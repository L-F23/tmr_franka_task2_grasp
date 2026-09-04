#!/usr/bin/env python3
"""Continuously retract and dump the tool, opening near 6 cm retreat."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

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
        speed: float, opened_position: float, progress, *,
        opening_start_position: float | None = None,
        gripper_open_steps: int = 1,
        gripper_open_step_dwell_s: float = 0.0,
    ) -> dict:
        """Tilt from the first waypoint, open at the configured retreat, then continue."""
        if gripper_open_steps > 1:
            eligible = [
                index
                for index, waypoint in enumerate(plan, 1)
                if float(start_x) - float(waypoint["position_m"][0])
                >= float(open_after_m) - 1e-6
            ]
            if not eligible:
                raise RuntimeError("release plan never reached the gripper-open distance")
            step_count = min(int(gripper_open_steps), len(eligible))
            schedule = np.rint(np.linspace(
                eligible[0], eligible[-1], step_count
            )).astype(int).tolist()
            opening_start = float(
                opened_position if opening_start_position is None
                else opening_start_position
            )
            targets = np.linspace(
                opening_start, float(opened_position), step_count + 1
            )[1:].tolist()
            scheduled_targets = dict(zip(schedule, targets))
            final_gripper = None
            started = False
            for index, waypoint in enumerate(plan, 1):
                result = self.move_ptp(
                    waypoint["joint_positions_rad"],
                    f"retract_dump_{index}_of_{len(plan)}",
                    speed,
                )
                retreat = float(start_x) - float(waypoint["position_m"][0])
                progress("motion", {**result, "retreat_m": retreat})
                if index not in scheduled_targets:
                    continue
                if not started:
                    progress("gripper_started", {"retreat_m": retreat})
                    started = True
                self.motion_gate()
                final_gripper = self.command_gripper(
                    scheduled_targets[index],
                    f"slow_open_step_{schedule.index(index) + 1}_of_{step_count}",
                )
                progress("gripper_step", {
                    **final_gripper,
                    "retreat_m": retreat,
                    "step": schedule.index(index) + 1,
                    "step_count": step_count,
                })
                if gripper_open_step_dwell_s > 0.0:
                    time.sleep(float(gripper_open_step_dwell_s))
            if final_gripper is None:
                raise RuntimeError("staged gripper opening did not execute")
            progress("gripper_finished", final_gripper)
            return final_gripper

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
    parser.add_argument("--backward-m", type=float, default=0.10)
    parser.add_argument("--initial-down-m", type=float, default=0.01)
    parser.add_argument("--down-m", type=float, default=0.035)
    parser.add_argument("--tilt-down-deg", type=float, default=20.0)
    parser.add_argument("--open-after-m", type=float, default=0.08)
    parser.add_argument(
        "--open-travel-fraction", type=float, default=0.4,
        help="fraction of the full closed-to-open gripper travel",
    )
    parser.add_argument("--gripper-open-steps", type=int, default=1)
    parser.add_argument("--gripper-open-step-dwell-s", type=float, default=0.0)
    parser.add_argument(
        "--defer-gripper-open", action="store_true",
        help="keep the gripper closed through this stage; a later lift stage must open it",
    )
    parser.add_argument(
        "--terminal-left-correction-m", type=float, default=0.0,
        help="calibrated base-frame +Y correction before the final extra tilt",
    )
    parser.add_argument(
        "--final-lift-m", type=float, default=-0.005,
        help="signed final finger-contact height change; negative moves down",
    )
    parser.add_argument("--final-extra-tilt-deg", type=float, default=25.0)
    parser.add_argument("--follow-through-inward-m", type=float, default=0.025)
    parser.add_argument("--follow-through-extra-tilt-deg", type=float, default=10.0)
    parser.add_argument("--follow-through-contact-z-delta-m", type=float, default=0.0)
    parser.add_argument("--pre-open-contact-drop-m", type=float, default=0.015)
    parser.add_argument("--maximum-contact-drop-m", type=float, default=0.045)
    parser.add_argument("--speed-rad-s", type=float, default=0.05)
    parser.add_argument("--descent-ease-power", type=float, default=2.0)
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required for physical motion")
    if not 0.10 <= args.backward_m <= 0.12:
        parser.error("--backward-m must be in [0.10, 0.12]")
    if not 0.0 <= args.down_m <= 0.07:
        parser.error("--down-m must be in [0, 0.07]")
    if not 0.0 < args.initial_down_m <= 0.02:
        parser.error("--initial-down-m must be in (0, 0.02]")
    if not 1.0 <= args.tilt_down_deg <= 90.0:
        parser.error("--tilt-down-deg must be in [1, 90]")
    if not 0.04 <= args.open_after_m < args.backward_m:
        parser.error("--open-after-m must be in [0.04, backward-m)")
    if not 0.0 < args.open_travel_fraction <= 1.0:
        parser.error("--open-travel-fraction must be in (0, 1]")
    if not 1 <= args.gripper_open_steps <= 8:
        parser.error("--gripper-open-steps must be in [1, 8]")
    if not 0.0 <= args.gripper_open_step_dwell_s <= 1.0:
        parser.error("--gripper-open-step-dwell-s must be in [0, 1]")
    if not 0.0 <= args.terminal_left_correction_m <= 0.02:
        parser.error("--terminal-left-correction-m must be in [0, 0.02]")
    if args.terminal_left_correction_m > 0.0 and not args.defer_gripper_open:
        parser.error("terminal left correction requires --defer-gripper-open")
    if not -0.01 <= args.final_lift_m <= 0.03:
        parser.error("--final-lift-m must be in [-0.01, 0.03]")
    if not 0.0 <= args.final_extra_tilt_deg <= 30.0:
        parser.error("--final-extra-tilt-deg must be in [0, 30]")
    if not 0.0 <= args.follow_through_inward_m <= 0.05:
        parser.error("--follow-through-inward-m must be in [0, 0.05]")
    if not 0.0 <= args.follow_through_extra_tilt_deg <= 20.0:
        parser.error("--follow-through-extra-tilt-deg must be in [0, 20]")
    if not -0.01 <= args.follow_through_contact_z_delta_m <= 0.02:
        parser.error("--follow-through-contact-z-delta-m must be in [-0.01, 0.02]")
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
        "gripper_motion": (
            "deferred_until_final_vertical_lift"
            if args.defer_gripper_open
            else "staged opening distributed across the remaining retreat"
        ),
        "open_after_retreat_m": args.open_after_m,
        "open_travel_fraction": args.open_travel_fraction,
        "gripper_open_steps": args.gripper_open_steps,
        "gripper_open_step_dwell_s": args.gripper_open_step_dwell_s,
        "terminal_left_correction_m": args.terminal_left_correction_m,
        "terminal_left_correction_source": "10mm wrist probe scaled by 1.2",
        "final_combined_lift_m": args.final_lift_m,
        "final_contact_z_delta_m": args.final_lift_m,
        "final_extra_down_tilt_deg": args.final_extra_tilt_deg,
        "follow_through_inward_m": args.follow_through_inward_m,
        "follow_through_extra_down_tilt_deg": args.follow_through_extra_tilt_deg,
        "follow_through_contact_z_delta_m": args.follow_through_contact_z_delta_m,
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
        if not args.defer_gripper_open and not node.gripper_client.wait_for_server(timeout_sec=5.0):
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
            "release_initial_down",
            np.asarray(start_position, dtype=float), lowered_start,
            orientation, orientation, node.joints,
        )
        previous_position = np.asarray(lowered_start, dtype=float)
        previous_orientation = np.asarray(orientation, dtype=float)
        clearance_compensations = []
        allowed_contact_drops = []
        descent_fractions = []
        open_fraction = args.open_after_m / args.backward_m
        # Ten milestones make translation and the configured dump simultaneous.
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
        fully_open = float(config["empty_cycle"]["open_position"])
        closed = float(config["empty_cycle"]["closed_position"])
        opened = closed + args.open_travel_fraction * (fully_open - closed)
        report["commanded_partial_open_position"] = opened
        def progress(kind, value):
            if kind == "motion":
                report["motions"].append(value)
            elif kind == "gripper_step":
                report.setdefault("gripper_steps", []).append(value)
            else:
                report[kind] = value
            args.record.parent.mkdir(parents=True, exist_ok=True)
            args.record.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

        if args.defer_gripper_open:
            for index, waypoint in enumerate(plan, 1):
                result = node.move_ptp(
                    waypoint["joint_positions_rad"],
                    f"retract_dump_{index}_of_{len(plan)}",
                    args.speed_rad_s,
                )
                progress("motion", {
                    **result,
                    "retreat_m": float(start_position[0])
                    - float(waypoint["position_m"][0]),
                })
            final_gripper = {
                "status": "held_closed",
                "open_deferred_to_final_vertical_lift": True,
            }
            progress("gripper_open_deferred", final_gripper)
        else:
            final_gripper = node.retract_tilt_and_open(
                plan, float(start_position[0]), args.open_after_m,
                args.speed_rad_s, opened, progress,
                opening_start_position=closed,
                gripper_open_steps=args.gripper_open_steps,
                gripper_open_step_dwell_s=args.gripper_open_step_dwell_s,
            )
        if args.terminal_left_correction_m > 0.0:
            corrected_position = np.asarray(previous_position, dtype=float) + np.array([
                0.0, args.terminal_left_correction_m, 0.0,
            ])
            correction_plan, _ = node.solve_pose_segment(
                "terminal_left_correction",
                np.asarray(previous_position, dtype=float), corrected_position,
                previous_orientation, previous_orientation, node.joints,
            )
            for index, waypoint in enumerate(correction_plan, 1):
                result = node.move_ptp(
                    waypoint["joint_positions_rad"],
                    f"terminal_left_correction_{index}_of_{len(correction_plan)}",
                    args.speed_rad_s,
                )
                progress("motion", {
                    **result,
                    "terminal_left_correction_m": float(
                        waypoint["position_m"][1] - previous_position[1]
                    ),
                })
            previous_position = corrected_position
            report["terminal_left_corrected_link8_base_position_m"] = (
                corrected_position.tolist()
            )
        final_orientation = tilt_axis_toward(
            tilted_orientation,
            motion["link8_gripper_approach_axis_local_xyz"],
            -np.asarray(motion["ground_up_axis_xyz"], dtype=float),
            args.final_extra_tilt_deg,
        ).tolist()
        # Rotate about the finger contact point, then apply the requested
        # signed contact-height delta without letting wrist rotation amplify it.
        contact_before = (
            np.asarray(previous_position, dtype=float)
            + quaternion_matrix(tilted_orientation) @ contact_local
        )
        final_position = (
            contact_before
            + np.array([0.0, 0.0, args.final_lift_m])
            - quaternion_matrix(final_orientation) @ contact_local
        )
        final_plan, _ = node.solve_pose_segment(
            "final_down_tilt_while_lifting",
            np.asarray(previous_position, dtype=float), final_position,
            tilted_orientation, final_orientation, node.joints,
        )
        for index, waypoint in enumerate(final_plan, 1):
            result = node.move_ptp(
                waypoint["joint_positions_rad"],
                f"final_down_tilt_lift_{index}_of_{len(final_plan)}",
                args.speed_rad_s,
            )
            progress("motion", {
                **result,
                "retreat_m": float(start_position[0]) - float(waypoint["position_m"][0]),
            })
        follow_orientation = tilt_axis_toward(
            final_orientation,
            motion["link8_gripper_approach_axis_local_xyz"],
            -np.asarray(motion["ground_up_axis_xyz"], dtype=float),
            args.follow_through_extra_tilt_deg,
        ).tolist()
        final_contact = (
            np.asarray(final_position, dtype=float)
            + quaternion_matrix(final_orientation) @ contact_local
        )
        follow_contact = final_contact + np.array([
            -args.follow_through_inward_m,
            0.0,
            args.follow_through_contact_z_delta_m,
        ])
        follow_position = (
            follow_contact - quaternion_matrix(follow_orientation) @ contact_local
        )
        follow_plan, _ = node.solve_pose_segment(
            "follow_through_inward_and_down_tilt",
            final_position, follow_position,
            final_orientation, follow_orientation, node.joints,
        )
        for index, waypoint in enumerate(follow_plan, 1):
            result = node.move_ptp(
                waypoint["joint_positions_rad"],
                f"follow_through_inward_tilt_{index}_of_{len(follow_plan)}",
                args.speed_rad_s,
            )
            progress("motion", {
                **result,
                "retreat_m": float(start_position[0]) - float(waypoint["position_m"][0]),
            })
        report["final_combined_target_link8_base_position_m"] = final_position.tolist()
        report["final_combined_target_link8_orientation_xyzw"] = final_orientation
        report["final_rotation_pivot"] = "finger_contact_point"
        report["follow_through_target_contact_base_m"] = follow_contact.tolist()
        report["follow_through_target_link8_base_position_m"] = follow_position.tolist()
        report["follow_through_target_link8_orientation_xyzw"] = follow_orientation
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
