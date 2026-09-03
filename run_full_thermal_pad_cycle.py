#!/usr/bin/env python3
"""Run the guarded 2 m approach and complete thermal-pad transfer cycle."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import time

from base_motion import guarded_transport


ROOT = Path(__file__).resolve().parent
DEFAULT_RECORD = ROOT / "config" / "latest_full_thermal_pad_cycle.json"
FULL_STAGE_ORDER = (
    "left_runtime_ready",
    "left_initial_verified_before_base_transport",
    "base_right_2m_complete",
    "black_base_and_thermal_pad_centered",
    "left_pregrasp_reached",
    "left_grasp_pose_reached",
    "thermal_pad_grasped",
    "left_arm_lifted_12cm",
    "red_pad_station_reached",
    "placement_forward_12cm_down_12cm_complete",
    "retract_11cm_down_1cm_complete_then_gripper_open",
    "left_initial_restored",
)


def run(command: list[str], label: str, timeout_s: float) -> dict:
    completed = subprocess.run(
        command,
        cwd=ROOT,
        env=os.environ.copy(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout_s,
        check=False,
    )
    output = completed.stdout or ""
    print(output, end="" if output.endswith("\n") else "\n", flush=True)
    if completed.returncode:
        raise RuntimeError(f"{label} failed with exit code {completed.returncode}")
    return {"label": label, "returncode": completed.returncode}


def python_command(script: str, *arguments: str) -> list[str]:
    return ["/usr/bin/python3", "-u", str(ROOT / script), *arguments]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--initial-right-m", type=float, default=2.0)
    parser.add_argument("--maximum-alignment-steps", type=int, default=20)
    parser.add_argument("--record", type=Path, default=DEFAULT_RECORD)
    args = parser.parse_args()
    if not args.execute:
        print(json.dumps({
            "status": "dry_run",
            "physical_motion_commanded": False,
            "initial_right_m": args.initial_right_m,
            "ordered_stages": FULL_STAGE_ORDER,
        }, indent=2))
        return 0
    if not 0.0 < args.initial_right_m <= 2.0:
        parser.error("--initial-right-m must be in (0, 2.0]")

    record = {
        "status": "running",
        "started_at_unix_s": time.time(),
        "execute_authorized": True,
        "requested_initial_right_m": args.initial_right_m,
        "ordered_stages": [],
        "stage_results": [],
        "base_transport_steps": [],
        "right_arm_commanded": False,
        "spine_commanded": False,
    }
    code = 2
    try:
        record["stage_results"].append(run(
            python_command("bootstrap_left_runtime.py", "--state-only"),
            "bootstrap_left_runtime",
            45.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[0])
        record["stage_results"].append(run(
            python_command("restore_left_initial_direct.py"),
            "restore_left_initial_before_base_transport",
            150.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[1])

        transport_steps, actual_transport_m = guarded_transport(args.initial_right_m)
        for result in transport_steps:
            record["base_transport_steps"].append(result)
            print(json.dumps({"base_transport_step": result}), flush=True)
        record["actual_initial_right_m"] = actual_transport_m
        record["ordered_stages"].append(FULL_STAGE_ORDER[2])

        record["stage_results"].append(run(
            python_command(
                "align_to_thermal_pad.py",
                "--execute",
                "--maximum-steps", str(args.maximum_alignment_steps),
            ),
            "black_base_visual_alignment",
            420.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[3])

        record["stage_results"].append(run(
            python_command(
                "execute_thermal_pad_grasp.py",
                "--execute", "--empty-cycle", "--fast",
                "--isolated-base-zero-locked", "--stage-start-only",
                "--record", str(ROOT / "config" / "latest_stage1_pregrasp.json"),
            ),
            "left_pregrasp",
            180.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[4])

        record["stage_results"].append(run(
            python_command(
                "set_stage1_start_from_current.py",
                "--execute", "--backward-m", "0", "--forward-m", "0.025",
                "--down-m", "0", "--up-m", "0",
                "--record", str(ROOT / "config" / "latest_stage1_start.json"),
            ),
            "left_grasp_pose",
            60.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[5])

        record["stage_results"].append(run(
            python_command("stage1_close_gripper.py", "--execute"),
            "thermal_pad_grasp",
            60.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[6])

        record["stage_results"].append(run(
            python_command(
                "set_stage1_start_from_current.py",
                "--execute", "--backward-m", "0", "--forward-m", "0",
                "--down-m", "0", "--up-m", "0.12",
                "--record", str(ROOT / "config" / "latest_stage2_lift.json"),
            ),
            "left_vertical_lift",
            90.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[7])

        record["stage_results"].append(run(
            python_command(
                "stage3_red_pad_alignment.py",
                "--execute", "--from-black-base-reference",
            ),
            "red_pad_station_alignment",
            180.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[8])

        record["stage_results"].append(run(
            python_command(
                "set_stage1_start_from_current.py",
                "--execute", "--backward-m", "0", "--forward-m", "0.12",
                "--down-m", "0.12", "--up-m", "0",
                "--record", str(ROOT / "config" / "latest_stage4_place_approach.json"),
            ),
            "placement_approach",
            150.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[9])

        record["stage_results"].append(run(
            python_command(
                "stage5_release_diagonal.py", "--execute",
                "--backward-m", "0.11", "--down-m", "0.01",
            ),
            "retract_down_then_release",
            120.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[10])

        record["stage_results"].append(run(
            python_command("restore_left_initial_direct.py"),
            "restore_left_initial_after_release",
            150.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[11])
        record["status"] = "complete"
        code = 0
    except Exception as exc:
        record["status"] = "blocked"
        record["error"] = str(exc)
    finally:
        record["completed_at_unix_s"] = time.time()
        args.record.parent.mkdir(parents=True, exist_ok=True)
        args.record.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(record, indent=2), flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
