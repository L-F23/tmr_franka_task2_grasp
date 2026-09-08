#!/usr/bin/env python3
"""Close the left gripper only at the freshly recorded stage-one pose."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy

from execute_thermal_pad_grasp import ThermalPadExecutor
from set_stage1_start_from_current import wait_motion_inputs
from thermal_pad_ik import DEFAULT_CONFIG, ROOT


DEFAULT_REFERENCE = ROOT / "config" / "latest_stage1_start.json"
DEFAULT_RECORD = ROOT / "config" / "latest_stage1_close.json"
DEFAULT_ALIGNMENT_RECORD = ROOT / "config" / "latest_pregrasp_lateral_alignment.json"


def reference_joints(record: dict) -> list[float]:
    if "after" in record:
        return list(map(float, record["after"]["joint_positions_rad"]))
    return list(map(float, record["joint_positions_rad"]))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--reference", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument("--record", type=Path, default=DEFAULT_RECORD)
    parser.add_argument("--alignment-record", type=Path, default=DEFAULT_ALIGNMENT_RECORD)
    parser.add_argument("--maximum-reference-error-rad", type=float, default=0.012)
    parser.add_argument(
        "--skip-pregrasp-calibration-gates",
        action="store_true",
        help="temporarily bypass visual-alignment freshness and grasp-pose recheck",
    )
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required for physical gripper motion")

    config = json.loads(args.config.read_text(encoding="utf-8"))
    report = {
        "status": "starting",
        "pregrasp_calibration_gates_enabled": (
            not args.skip_pregrasp_calibration_gates
        ),
        "base_commanded": False,
        "right_arm_commanded": False,
        "spine_commanded": False,
    }
    expected = None
    if args.skip_pregrasp_calibration_gates:
        report["pregrasp_calibration_gate_status"] = "temporarily_bypassed"
    else:
        reference = json.loads(args.reference.read_text(encoding="utf-8"))
        alignment = json.loads(args.alignment_record.read_text(encoding="utf-8"))
        aligned_at_unix_s = float(alignment.get("aligned_at_unix_s", 0.0))
        alignment_age_s = time.time() - aligned_at_unix_s
        # Validate that alignment was fresh when the guarded approach began.
        # The approach/contact/retreat itself can legitimately take longer than
        # the freshness window and does not move the base.
        approach_started_at_unix_s = float(
            reference.get("before", {}).get("recorded_unix_s", time.time())
        )
        alignment_age_at_approach_start_s = (
            approach_started_at_unix_s - aligned_at_unix_s
        )
        maximum_alignment_age_s = float(json.loads(
            (ROOT / "config" / "pregrasp_lateral_alignment.json").read_text(encoding="utf-8")
        )["maximum_alignment_record_age_s"])
        if alignment.get("status") != "pregrasp_lateral_alignment_confirmed":
            raise RuntimeError("mandatory pregrasp lateral alignment is not confirmed")
        if not 0.0 <= alignment_age_at_approach_start_s <= maximum_alignment_age_s:
            raise RuntimeError(
                "mandatory pregrasp alignment was stale when approach started: "
                f"{alignment_age_at_approach_start_s:.3f}s"
            )
        if reference.get("base_commanded") is not False:
            raise RuntimeError("stage-one approach record does not confirm a stationary base")
        expected = np.asarray(reference_joints(reference), dtype=float)
        report.update(
            reference=str(args.reference),
            alignment_record=str(args.alignment_record),
            alignment_age_s=alignment_age_s,
            alignment_age_at_approach_start_s=alignment_age_at_approach_start_s,
        )
    rclpy.init()
    node = ThermalPadExecutor(config)
    node.isolated_base_zero_locked = True
    code = 2
    try:
        if not node.gripper_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("left gripper action unavailable")
        wait_motion_inputs(node)
        node.motion_gate()
        if expected is not None:
            measured = np.asarray(node.joints, dtype=float)
            reference_error = float(np.max(np.abs(measured - expected)))
            report.update(
                measured_joint_positions_rad=measured.tolist(),
                expected_joint_positions_rad=expected.tolist(),
                maximum_reference_error_rad=reference_error,
            )
            if reference_error > args.maximum_reference_error_rad:
                raise RuntimeError(
                    f"left arm moved away from recorded grasp pose: "
                    f"{reference_error:.6f} rad"
                )
        result = node.command_gripper(
            float(config["empty_cycle"]["closed_position"]),
            "close_at_recorded_thermal_pad_pose",
        )
        report["gripper"] = result
        if not (result["reached_goal"] or result["stalled"]):
            raise RuntimeError("left gripper neither reached the goal nor stalled on contact")
        report["status"] = "stage_1_grasp_complete"
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
