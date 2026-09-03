#!/usr/bin/env python3
"""Move the left link8 along ground-aligned axes and record the measured result."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from franka_spine_msgs.srv import GetPosition

from execute_thermal_pad_grasp import ThermalPadExecutor
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


def wait_motion_inputs(node: ThermalPadExecutor, timeout_s: float = 10.0) -> None:
    for service in (node.fk_client, node.ik_client, node.validity_client, node.spine_client):
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
    }
    code = 2
    try:
        if not node.ptp_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("left PTP action unavailable")
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
        if args.backward_m > 0.0 or args.forward_m > 0.0:
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
        x_label = "forward" if args.forward_m > 0.0 else "backward"
        z_label = "up" if args.up_m > 0.0 else "down"
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
