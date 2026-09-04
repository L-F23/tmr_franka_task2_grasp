#!/usr/bin/env python3
"""Run Task 2 from the left-arm reset pose through grasp, transfer, and restore.

The robot services must already be running.  This entry does not perform the
initial 2 m base transport, black-base search, or table-edge alignment.  It
first restores the left arm to the recorded Task 2 initial joints, immediately
moves it to the calibrated pregrasp pose, verifies that pose, and only then
continues with the verified grasp and transfer sequence.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from run_from_pregrasp_to_finish import (
    ROOT,
    execute,
    python_command,
    stage_commands as pregrasp_stage_commands,
)
from mission_runtime import (
    MissionAlreadyRunning,
    acquire_motion_lock,
    release_motion_lock,
)


DEFAULT_RECORD = ROOT / "config" / "latest_task2_from_initial.json"

STAGE_ORDER = (
    "services_and_live_cameras_verified",
    "left_initial_restored",
    "left_pregrasp_reached",
    "calibrated_pregrasp_pose_verified",
    "grasp_pose_reached_2_5cm_forward",
    "pregrasp_lateral_alignment_confirmed",
    "thermal_pad_grasped",
    "left_arm_lifted_12cm",
    "red_pad_centered_under_left_wrist",
    "placement_forward_12cm_down_12cm_complete",
    "release_retract_tilt_and_open_complete",
    "post_release_vertical_clearance_5cm",
    "left_initial_restored_after_transfer",
)


def stage_commands() -> tuple[tuple[str, list[str], float], ...]:
    continuation = pregrasp_stage_commands()
    # Reuse the previously tested stages, replacing only their initial
    # assumptions with the explicit reset -> pregrasp transition.
    return (
        (
            STAGE_ORDER[0],
            python_command("quick_start.py", "--check-only"),
            30.0,
        ),
        (
            STAGE_ORDER[1],
            python_command("restore_left_initial_direct.py"),
            150.0,
        ),
        (
            STAGE_ORDER[2],
            python_command(
                "execute_thermal_pad_grasp.py",
                "--execute", "--empty-cycle", "--fast",
                "--isolated-base-zero-locked", "--stage-start-only",
                "--record", str(ROOT / "config" / "latest_stage1_pregrasp.json"),
            ),
            180.0,
        ),
        # Skip the pregrasp-only runner's health check, but retain its
        # independent measured-joint/FK validation and every later stage.
        *continuation[1:-1],
        (
            STAGE_ORDER[-1],
            python_command("restore_left_initial_direct.py"),
            150.0,
        ),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--record", type=Path, default=DEFAULT_RECORD)
    args = parser.parse_args()
    stages = stage_commands()
    if not args.execute:
        print(json.dumps({
            "status": "dry_run",
            "physical_motion_commanded": False,
            "services_started": False,
            "entry_condition": "Task 2 starts from the left-arm reset pose",
            "ordered_stages": [label for label, _command, _timeout in stages],
        }, indent=2))
        return 0

    lock = None
    try:
        lock = acquire_motion_lock()
        return execute(
            args.record,
            stages=stages,
            entry_condition=(
                "Task 2 left-arm reset pose; base already at the black-base reference"
            ),
            excluded_stages=[
                "robot_service_startup",
                "initial_2m_base_transport",
                "black_base_coarse_search",
                "table_edge_fore_aft_alignment",
            ],
        )
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
