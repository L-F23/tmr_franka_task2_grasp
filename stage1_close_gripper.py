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
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required for physical gripper motion")

    config = json.loads(args.config.read_text(encoding="utf-8"))
    reference = json.loads(args.reference.read_text(encoding="utf-8"))
    alignment = json.loads(args.alignment_record.read_text(encoding="utf-8"))
    alignment_age_s = time.time() - float(alignment.get("aligned_at_unix_s", 0.0))
    maximum_alignment_age_s = float(json.loads(
        (ROOT / "config" / "pregrasp_lateral_alignment.json").read_text(encoding="utf-8")
    )["maximum_alignment_record_age_s"])
    if alignment.get("status") != "pregrasp_lateral_alignment_confirmed":
        raise RuntimeError("mandatory pregrasp lateral alignment is not confirmed")
    if not 0.0 <= alignment_age_s <= maximum_alignment_age_s:
        raise RuntimeError(
            f"mandatory pregrasp alignment record is stale: {alignment_age_s:.3f}s"
        )
    expected = np.asarray(reference_joints(reference), dtype=float)
    report = {
        "status": "starting",
        "reference": str(args.reference),
        "alignment_record": str(args.alignment_record),
        "alignment_age_s": alignment_age_s,
        "base_commanded": False,
        "right_arm_commanded": False,
        "spine_commanded": False,
    }
    rclpy.init()
    node = ThermalPadExecutor(config)
    node.isolated_base_zero_locked = True
    code = 2
    try:
        if not node.gripper_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("left gripper action unavailable")
        wait_motion_inputs(node)
        node.motion_gate()
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
