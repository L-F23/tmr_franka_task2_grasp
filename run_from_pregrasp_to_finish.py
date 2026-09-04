#!/usr/bin/env python3
"""Run Task 2 from the calibrated left-arm pregrasp pose through final restore.

This entry deliberately does not start robot services and does not perform the
initial 2 m transport, black-base search, table-edge alignment, or motion into
the pregrasp pose.  Every stage is fail-closed and the first physical command
is issued only after read-only service/camera and pose gates pass.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import time

from mission_runtime import (
    LOCK_FILE,
    MissionAlreadyRunning,
    acquire_motion_lock,
    atomic_write_json,
    release_motion_lock,
)


ROOT = Path(__file__).resolve().parent
DEFAULT_RECORD = ROOT / "config" / "latest_pregrasp_to_finish.json"

# Fast physical profile validated against the deployed FR3 PTP action.  Keep
# the near-table dump slower than free-space translation while avoiding the
# former blanket 0.05 rad/s setting.
FAST_ARM_SPEED_RAD_S = "0.08"
FAST_RELEASE_SPEED_RAD_S = "0.065"

STAGE_ORDER = (
    "services_and_live_cameras_verified",
    "spine_restored_0_6m",
    "calibrated_pregrasp_pose_verified",
    "pregrasp_lateral_alignment_confirmed",
    "force_contact_then_retreat_18mm",
    "thermal_pad_grasped",
    "left_arm_lifted_12cm",
    "red_pad_station_reached_main_camera_wrist_advisory",
    "placement_forward_143mm_down_12cm_complete",
    "placement_retract_tilt_with_gripper_held",
    "gripper_open_then_vertical_clearance_5cm",
    "left_initial_restored",
)


def python_command(script: str, *arguments: str) -> list[str]:
    return ["/usr/bin/python3", "-u", str(ROOT / script), *arguments]


def stage_commands() -> tuple[tuple[str, list[str], float], ...]:
    return (
        (
            STAGE_ORDER[0],
            python_command("quick_start.py", "--parent-lock-held", "--check-only"),
            30.0,
        ),
        (
            STAGE_ORDER[1],
            python_command("reset_spine_to_task_height.py", "--execute"),
            620.0,
        ),
        (
            STAGE_ORDER[2],
            python_command("verify_pregrasp_ready.py"),
            30.0,
        ),
        (
            STAGE_ORDER[3],
            python_command("black_base_pose_alignment.py", "--execute"),
            360.0,
        ),
        (
            STAGE_ORDER[4],
            python_command(
                "set_stage1_start_from_current.py",
                "--execute", "--backward-m", "0", "--forward-m", "0.162",
                "--down-m", "0", "--up-m", "0",
                "--speed-rad-s", "0.025", "--fast",
                "--guarded-contact-approach", "--contact-step-m", "0.002",
                "--axis-force-delta-n", "2.5",
                "--force-delta-norm-n", "4.0",
                "--torque-delta-norm-nm", "1.5",
                "--joint-torque-delta-nm", "2.0",
                "--contact-consecutive-samples", "5",
                "--contact-retreat-m", "0.018",
                "--record", str(ROOT / "config" / "latest_stage1_start.json"),
            ),
            120.0,
        ),
        (
            STAGE_ORDER[5],
            python_command(
                "stage1_close_gripper.py", "--execute",
            ),
            60.0,
        ),
        (
            STAGE_ORDER[6],
            python_command(
                "set_stage1_start_from_current.py",
                "--execute", "--backward-m", "0", "--forward-m", "0",
                "--down-m", "0", "--up-m", "0.12",
                "--speed-rad-s", FAST_ARM_SPEED_RAD_S, "--fast",
                "--record", str(ROOT / "config" / "latest_stage2_lift.json"),
            ),
            90.0,
        ),
        (
            STAGE_ORDER[7],
            python_command(
                "stage3_red_pad_alignment.py",
                "--execute", "--from-black-base-reference",
            ),
            240.0,
        ),
        (
            STAGE_ORDER[8],
            python_command(
                "set_stage1_start_from_current.py",
                "--execute", "--backward-m", "0", "--forward-m", "0.143",
                "--down-m", "0.12", "--up-m", "0",
                "--speed-rad-s", FAST_ARM_SPEED_RAD_S, "--fast",
                "--record", str(ROOT / "config" / "latest_stage4_place_approach.json"),
            ),
            150.0,
        ),
        (
            STAGE_ORDER[9],
            python_command(
                "stage5_release_diagonal.py", "--execute",
                "--backward-m", "0.10", "--initial-down-m", "0.015",
                "--down-m", "0.035", "--pre-open-contact-drop-m", "0.015",
                "--maximum-contact-drop-m", "0.05",
                "--tilt-down-deg", "20", "--open-after-m", "0.08",
                "--open-travel-fraction", "0.4", "--defer-gripper-open",
                "--terminal-left-correction-m", "0.012",
                "--final-lift-m", "-0.005", "--final-extra-tilt-deg", "25",
                "--follow-through-inward-m", "0.020",
                "--follow-through-extra-tilt-deg", "10",
                "--follow-through-contact-z-delta-m", "0",
                "--speed-rad-s", FAST_RELEASE_SPEED_RAD_S,
            ),
            120.0,
        ),
        (
            STAGE_ORDER[10],
            python_command(
                "set_stage1_start_from_current.py",
                "--execute", "--backward-m", "0", "--forward-m", "0",
                "--down-m", "0", "--up-m", "0.05",
                "--open-gripper-before-motion",
                "--speed-rad-s", FAST_ARM_SPEED_RAD_S, "--fast",
                "--record", str(ROOT / "config" / "latest_post_release_clearance.json"),
            ),
            90.0,
        ),
        (
            STAGE_ORDER[11],
            python_command("restore_left_initial_direct.py"),
            150.0,
        ),
    )


def write_record(path: Path, record: dict) -> None:
    atomic_write_json(path, record)


def run_stage(label: str, command: list[str], timeout_s: float) -> dict:
    print(json.dumps({"stage": label, "status": "starting"}), flush=True)
    started = time.monotonic()
    completed = subprocess.run(
        command,
        cwd=ROOT,
        env=os.environ.copy(),
        timeout=timeout_s,
        check=False,
    )
    if completed.returncode:
        raise RuntimeError(f"{label} failed with exit code {completed.returncode}")
    return {
        "label": label,
        "returncode": completed.returncode,
        "elapsed_s": round(time.monotonic() - started, 3),
    }


def execute(
    record_path: Path,
    *,
    stages: tuple[tuple[str, list[str], float], ...] | None = None,
    entry_condition: str = (
        "left arm already at calibrated pregrasp pose and base at black-base reference"
    ),
    excluded_stages: list[str] | None = None,
) -> int:
    selected_stages = stage_commands() if stages is None else stages
    selected_exclusions = [
        "robot_service_startup",
        "initial_2m_base_transport",
        "black_base_coarse_search",
        "table_edge_fore_aft_alignment",
        "move_left_arm_into_pregrasp",
    ] if excluded_stages is None else excluded_stages
    record = {
        "schema_version": 1,
        "status": "running",
        "started_at_unix_s": time.time(),
        "entry_condition": entry_condition,
        "excluded_stages": selected_exclusions,
        "ordered_stages": [label for label, _command, _timeout in selected_stages],
        "physical_motion_authorized": True,
        "completed_stages": [],
        "stage_results": [],
        "right_arm_commanded": False,
        "spine_commanded": True,
    }
    write_record(record_path, record)
    code = 2
    try:
        for label, command, timeout_s in selected_stages:
            record["active_stage"] = label
            write_record(record_path, record)
            result = run_stage(label, command, timeout_s)
            record["stage_results"].append(result)
            record["completed_stages"].append(label)
            record.pop("active_stage", None)
            write_record(record_path, record)
        record["status"] = "complete"
        code = 0
    except KeyboardInterrupt:
        record["status"] = "interrupted"
        record["error"] = "operator interrupt"
        code = 130
    except (OSError, subprocess.TimeoutExpired, RuntimeError) as exc:
        record["status"] = "blocked"
        record["error"] = str(exc)
    finally:
        record["finished_at_unix_s"] = time.time()
        write_record(record_path, record)
        print(json.dumps(record, indent=2), flush=True)
    return code


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--record", type=Path, default=DEFAULT_RECORD)
    args = parser.parse_args()
    if not args.execute:
        print(json.dumps({
            "status": "dry_run",
            "physical_motion_commanded": False,
            "services_started": False,
            "entry_condition": "left arm already at calibrated pregrasp pose",
            "ordered_stages": STAGE_ORDER,
        }, indent=2))
        return 0

    lock = None
    try:
        lock = acquire_motion_lock()
        return execute(args.record)
    except MissionAlreadyRunning as exc:
        print(json.dumps({
            "status": "blocked",
            "error": str(exc),
            "physical_motion_commanded": False,
        }, indent=2))
        return 73
    finally:
        release_motion_lock(lock)


if __name__ == "__main__":
    raise SystemExit(main())
