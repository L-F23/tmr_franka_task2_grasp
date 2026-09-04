#!/usr/bin/env python3
"""Run Task 2 from the calibrated left-arm pregrasp pose through final restore.

This entry deliberately does not start robot services and does not perform the
initial 2 m transport, black-base search, table-edge alignment, or motion into
the pregrasp pose.  Every stage is fail-closed and the first physical command
is issued only after read-only service/camera and pose gates pass.
"""

from __future__ import annotations

import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess
import time


ROOT = Path(__file__).resolve().parent
DEFAULT_RECORD = ROOT / "config" / "latest_pregrasp_to_finish.json"
LOCK_FILE = Path("/tmp/tmr_task2_pregrasp_to_finish.lock")

STAGE_ORDER = (
    "services_and_live_cameras_verified",
    "calibrated_pregrasp_pose_verified",
    "grasp_pose_reached_2_5cm_forward",
    "pregrasp_lateral_alignment_confirmed",
    "thermal_pad_grasped",
    "left_arm_lifted_12cm",
    "red_pad_centered_under_left_wrist",
    "placement_forward_12cm_down_12cm_complete",
    "release_retract_tilt_and_open_complete",
    "post_release_vertical_clearance_5cm",
    "left_initial_restored",
)


def python_command(script: str, *arguments: str) -> list[str]:
    return ["/usr/bin/python3", "-u", str(ROOT / script), *arguments]


def stage_commands() -> tuple[tuple[str, list[str], float], ...]:
    return (
        (
            STAGE_ORDER[0],
            python_command("quick_start.py", "--check-only"),
            30.0,
        ),
        (
            STAGE_ORDER[1],
            python_command("verify_pregrasp_ready.py"),
            30.0,
        ),
        (
            STAGE_ORDER[2],
            python_command(
                "set_stage1_start_from_current.py",
                "--execute", "--backward-m", "0", "--forward-m", "0.025",
                "--down-m", "0", "--up-m", "0",
                "--speed-rad-s", "0.05", "--fast",
                "--record", str(ROOT / "config" / "latest_stage1_start.json"),
            ),
            60.0,
        ),
        (
            STAGE_ORDER[3],
            python_command("pregrasp_lateral_alignment.py", "--execute"),
            360.0,
        ),
        (
            STAGE_ORDER[4],
            python_command("stage1_close_gripper.py", "--execute"),
            60.0,
        ),
        (
            STAGE_ORDER[5],
            python_command(
                "set_stage1_start_from_current.py",
                "--execute", "--backward-m", "0", "--forward-m", "0",
                "--down-m", "0", "--up-m", "0.12",
                "--speed-rad-s", "0.05", "--fast",
                "--record", str(ROOT / "config" / "latest_stage2_lift.json"),
            ),
            90.0,
        ),
        (
            STAGE_ORDER[6],
            python_command(
                "stage3_red_pad_alignment.py",
                "--execute", "--from-black-base-reference", "--wrist-closed-loop",
            ),
            240.0,
        ),
        (
            STAGE_ORDER[7],
            python_command(
                "set_stage1_start_from_current.py",
                "--execute", "--backward-m", "0", "--forward-m", "0.12",
                "--down-m", "0.12", "--up-m", "0",
                "--speed-rad-s", "0.05", "--fast",
                "--record", str(ROOT / "config" / "latest_stage4_place_approach.json"),
            ),
            150.0,
        ),
        (
            STAGE_ORDER[8],
            python_command(
                "stage5_release_diagonal.py", "--execute",
                "--backward-m", "0.11", "--initial-down-m", "0.008",
                "--down-m", "0.062", "--pre-open-contact-drop-m", "0.015",
                "--maximum-contact-drop-m", "0.07",
                "--tilt-down-deg", "90", "--open-after-m", "0.06",
                "--speed-rad-s", "0.05",
            ),
            120.0,
        ),
        (
            STAGE_ORDER[9],
            python_command(
                "set_stage1_start_from_current.py",
                "--execute", "--backward-m", "0", "--forward-m", "0",
                "--down-m", "0", "--up-m", "0.05",
                "--speed-rad-s", "0.05", "--fast",
                "--record", str(ROOT / "config" / "latest_post_release_clearance.json"),
            ),
            90.0,
        ),
        (
            STAGE_ORDER[10],
            python_command("restore_left_initial_direct.py"),
            150.0,
        ),
    )


def write_record(path: Path, record: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


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
        "spine_commanded": False,
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

    LOCK_FILE.touch(exist_ok=True)
    with LOCK_FILE.open("r+") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print(json.dumps({
                "status": "blocked",
                "error": "another pregrasp-to-finish run is active",
                "physical_motion_commanded": False,
            }, indent=2))
            return 73
        return execute(args.record)


if __name__ == "__main__":
    raise SystemExit(main())
