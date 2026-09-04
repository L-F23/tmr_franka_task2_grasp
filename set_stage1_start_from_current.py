#!/usr/bin/env python3
"""Move the left link8 along ground-aligned axes and record the measured result."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from franka_spine_msgs.srv import GetPosition

from execute_thermal_pad_grasp import GuardedContactStop, ThermalPadExecutor
from thermal_pad_ik import DEFAULT_CONFIG, ROOT, pose_values


DEFAULT_RECORD = ROOT / "config" / "latest_stage1_start.json"


def offset_targets(
    position, backward_m: float, down_m: float,
    forward_m: float = 0.0, up_m: float = 0.0,
):
    start = np.asarray(position, dtype=float)
    after_x = start + np.array([float(forward_m) - float(backward_m), 0.0, 0.0])
    after_z = after_x + np.array([0.0, 0.0, float(up_m) - float(down_m)])
    return after_x, after_z


def contact_retreat_target(position, retreat_m: float) -> np.ndarray:
    """Retract from measured contact along the ground-aligned approach axis."""
    return np.asarray(position, dtype=float) - np.array([float(retreat_m), 0.0, 0.0])


def wait_motion_inputs(node: ThermalPadExecutor, timeout_s: float = 10.0) -> None:
    for service in node.planning_services():
        if not service.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"service unavailable: {service.srv_name}")
    spine = node.call(node.spine_client, GetPosition.Request())
    if not spine.success:
        raise RuntimeError("spine position query failed")
    node.spine_position = float(spine.position)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.joints is not None and node.robot_state is not None and node.measured_pose is not None:
            return
    raise RuntimeError("fresh left-arm state unavailable")


def pose_record(node: ThermalPadExecutor) -> dict:
    pose = node.fk(node.joints)
    position, orientation = pose_values(pose)
    return {
        "recorded_unix_s": time.time(),
        "joint_positions_rad": list(node.joints),
        "link8_base_position_m": position,
        "link8_base_orientation_xyzw": orientation,
        "spine_position_m": float(node.spine_position),
        "active_errors": node.active_errors(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--record", type=Path, default=DEFAULT_RECORD)
    parser.add_argument("--backward-m", type=float, default=0.06)
    parser.add_argument("--forward-m", type=float, default=0.0)
    parser.add_argument("--down-m", type=float, default=0.055)
    parser.add_argument("--up-m", type=float, default=0.0)
    parser.add_argument("--speed-rad-s", type=float, default=0.05)
    parser.add_argument("--fast", action="store_true")
    parser.add_argument(
        "--guarded-contact-approach",
        action="store_true",
        help="open the gripper, then execute forward motion in force-guarded increments",
    )
    parser.add_argument("--contact-step-m", type=float, default=0.002)
    parser.add_argument("--axis-force-delta-n", type=float, default=4.0)
    parser.add_argument("--force-delta-norm-n", type=float, default=7.0)
    parser.add_argument("--torque-delta-norm-nm", type=float, default=1.5)
    parser.add_argument("--joint-torque-delta-nm", type=float, default=2.0)
    parser.add_argument("--contact-consecutive-samples", type=int, default=3)
    parser.add_argument("--contact-retreat-m", type=float, default=0.018)
    parser.add_argument(
        "--open-gripper-before-motion", action="store_true",
        help="open the left gripper after planning and immediately before this motion",
    )
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required for physical motion")
    if min(args.backward_m, args.forward_m, args.down_m, args.up_m) < 0.0:
        parser.error("offset distances must be non-negative")
    if args.backward_m > 0.0 and args.forward_m > 0.0:
        parser.error("choose either --backward-m or --forward-m")
    if args.down_m > 0.0 and args.up_m > 0.0:
        parser.error("choose either --down-m or --up-m")
    if (
        args.backward_m == 0.0 and args.forward_m == 0.0
        and args.down_m == 0.0 and args.up_m == 0.0
    ):
        parser.error("at least one offset distance must be positive")
    if args.guarded_contact_approach and not (
        args.forward_m > 0.0
        and args.backward_m == 0.0
        and args.down_m == 0.0
        and args.up_m == 0.0
    ):
        parser.error("guarded contact approach supports forward-only motion")
    if args.guarded_contact_approach and not 0.0005 <= args.contact_step_m <= 0.003:
        parser.error("guarded contact step must be between 0.5 mm and 3 mm")
    if args.guarded_contact_approach and not 0.001 <= args.contact_retreat_m <= 0.02:
        parser.error("contact retreat must be between 1 mm and 20 mm")

    config = json.loads(args.config.read_text(encoding="utf-8"))
    rclpy.init()
    node = ThermalPadExecutor(config)
    node.fast_execution = bool(args.fast)
    node.isolated_base_zero_locked = True
    report = {
        "status": "starting",
        "coordinate_frame": config["kinematics"]["frame"],
        "requested_backward_m": args.backward_m,
        "requested_forward_m": args.forward_m,
        "requested_down_m": args.down_m,
        "requested_up_m": args.up_m,
        "base_commanded": False,
        "right_arm_commanded": False,
        "motions": [],
        "guarded_contact_approach": bool(args.guarded_contact_approach),
    }
    code = 2
    try:
        if not node.ptp_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("left PTP action unavailable")
        if (args.guarded_contact_approach or args.open_gripper_before_motion) and not node.gripper_client.wait_for_server(
            timeout_sec=5.0
        ):
            raise RuntimeError("left gripper action unavailable")
        wait_motion_inputs(node)
        report["before"] = pose_record(node)
        if report["before"]["active_errors"]:
            raise RuntimeError("Franka errors: " + ",".join(report["before"]["active_errors"]))

        start_position = np.asarray(report["before"]["link8_base_position_m"], dtype=float)
        orientation = report["before"]["link8_base_orientation_xyzw"]
        after_x, after_z = offset_targets(
            start_position, args.backward_m, args.down_m, args.forward_m, args.up_m
        )
        x_plan, seed = [], list(node.joints)
        if (
            (args.backward_m > 0.0 or args.forward_m > 0.0)
            and not args.guarded_contact_approach
        ):
            direction = "forward" if args.forward_m > 0.0 else "backward"
            x_plan, seed = node.solve_cartesian_segment(
                f"stage1_start_{direction}", start_position, after_x, orientation, seed
            )
        z_plan = []
        if args.down_m > 0.0 or args.up_m > 0.0:
            z_direction = "up" if args.up_m > 0.0 else "down"
            z_plan, _ = node.solve_cartesian_segment(
                f"stage1_start_{z_direction}", after_x, after_z, orientation, seed
            )
        report["planned_targets"] = {
            "after_x_translation_m": after_x.tolist(),
            "after_z_translation_m": after_z.tolist(),
            "orientation_xyzw": orientation,
        }
        if args.open_gripper_before_motion:
            report["gripper_open_before_motion"] = node.command_gripper(
                float(config["empty_cycle"].get("open_position", 0.0)),
                "open_immediately_before_final_lift",
            )
            if not report["gripper_open_before_motion"]["reached_goal"]:
                raise RuntimeError("left gripper did not open before final lift")
        x_label = "forward" if args.forward_m > 0.0 else "backward"
        z_label = "up" if args.up_m > 0.0 else "down"
        if args.guarded_contact_approach:
            report["gripper_open"] = node.command_gripper(
                float(config["empty_cycle"].get("open_position", 0.0)),
                "open_before_guarded_approach",
            )
            if not report["gripper_open"]["reached_goal"]:
                raise RuntimeError("left gripper did not reach the open position")
            report["external_wrench_baseline"] = node.capture_external_wrench_baseline(
                approach_axis=0,
                axis_force_delta_n=args.axis_force_delta_n,
                force_delta_norm_n=args.force_delta_norm_n,
                torque_delta_norm_nm=args.torque_delta_norm_nm,
                joint_torque_delta_nm=args.joint_torque_delta_nm,
                consecutive_samples=args.contact_consecutive_samples,
            )
            increment_count = int(math.ceil(args.forward_m / args.contact_step_m))
            cursor_position = start_position.copy()
            contact_detected = False
            try:
                for increment in range(1, increment_count + 1):
                    travelled = min(args.forward_m, increment * args.contact_step_m)
                    target = start_position + np.array([travelled, 0.0, 0.0])
                    plan, seed = node.solve_cartesian_segment(
                        f"guarded_forward_{increment}",
                        cursor_position,
                        target,
                        orientation,
                        seed,
                    )
                    for waypoint in plan:
                        report["motions"].append(node.move_ptp(
                            waypoint["joint_positions_rad"],
                            f"guarded_forward_{increment}_{waypoint['index']}",
                            args.speed_rad_s,
                        ))
                    cursor_position = target
            except GuardedContactStop:
                contact_detected = True
                report["external_contact_stop"] = node.contact_stop
                node.disable_contact_guard()
                report["post_cancel_stationary"] = node.wait_arm_stationary()
                errors_after_contact = node.active_errors()
                if errors_after_contact:
                    raise RuntimeError(
                        "Franka error after contact stop; automatic retract blocked: "
                        + ",".join(errors_after_contact)
                    )
                contact_pose = pose_record(node)
                report["contact_pose"] = contact_pose
                retreat_position = contact_retreat_target(
                    contact_pose["link8_base_position_m"], args.contact_retreat_m
                )
                retreat_plan, _ = node.solve_cartesian_segment(
                    "post_contact_retreat",
                    np.asarray(contact_pose["link8_base_position_m"], dtype=float),
                    retreat_position,
                    contact_pose["link8_base_orientation_xyzw"],
                    list(node.joints),
                )
                for waypoint in retreat_plan:
                    report["motions"].append(node.move_ptp(
                        waypoint["joint_positions_rad"],
                        f"post_contact_retreat_{waypoint['index']}",
                        min(args.speed_rad_s, 0.015),
                    ))
                report["post_contact_retreat_m"] = float(args.contact_retreat_m)
            finally:
                node.disable_contact_guard()
            if not contact_detected:
                raise RuntimeError(
                    "no external contact detected within guarded approach limit; "
                    "gripper remains open"
                )
            report["guarded_contact_then_retreat_complete"] = True
        else:
            for label, plan in ((x_label, x_plan), (z_label, z_plan)):
                for waypoint in plan:
                    report["motions"].append(node.move_ptp(
                        waypoint["joint_positions_rad"],
                        f"stage1_start_{label}_{waypoint['index']}",
                        args.speed_rad_s,
                    ))
        report["after"] = pose_record(node)
        report["recommended_retracted_link8_position_m"] = report["after"][
            "link8_base_position_m"
        ]
        report["status"] = "relative_motion_complete"
        code = 0
    except Exception as exc:
        report["status"] = "blocked"
        report["error"] = str(exc)
        if node.contact_stop is not None:
            report["external_contact_stop"] = node.contact_stop
        if node.joints is not None and node.spine_position is not None:
            try:
                report["stopped_at"] = pose_record(node)
            except Exception as record_exc:
                report["stopped_at_error"] = str(record_exc)
    finally:
        args.record.parent.mkdir(parents=True, exist_ok=True)
        args.record.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
