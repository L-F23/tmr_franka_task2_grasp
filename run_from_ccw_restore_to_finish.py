#!/usr/bin/env python3
"""Run Task 2 from the pending CCW restore turn through final arm restore.

Entry condition: the left arm is already at the calibrated pregrasp pose, the
gripper is open, and the preceding clockwise turn has completed.  Spine and
the right-arm parking pose are restored before the first base motion, which is
the 90 degree CCW restore.
"""

from __future__ import annotations

import argparse
from argparse import Namespace
import json
from pathlib import Path
import time

from base_motion import (
    guarded_move_forward_continuous,
    guarded_move_right_continuous,
)
from mission_runtime import (
    MissionAlreadyRunning,
    acquire_motion_lock,
    atomic_write_json,
    release_motion_lock,
)
from run_from_pregrasp_to_finish import (
    execute as execute_finish,
    python_command,
    run_stage,
    stage_commands as finish_stage_commands,
)
from search_black_base_from_pregrasp import (
    DEFAULT_REFERENCE,
    execute as execute_target_search,
)
from stage0_wall_dock_and_find_stand import run_base_controller


ROOT = Path(__file__).resolve().parent
DEFAULT_RECORD = ROOT / "config" / "latest_ccw_restore_to_finish.json"
DEFAULT_SEARCH_RECORD = ROOT / "config" / "latest_ccw_route_target_search.json"
DEFAULT_FINISH_RECORD = ROOT / "config" / "latest_ccw_route_grasp_finish.json"

RESTORE_CCW_DEG = 90.0
BACKWARD_M = 0.55
PRESEARCH_RIGHT_M = 1.40
MAXIMUM_SEARCH_RIGHT_M = 1.50
REAR_CLEARANCE_M = 0.14518819414515705
REAR_WALL_ANGLE_DEG = 1.136309

ROUTE_STAGES = (
    "services_and_cameras_preflight",
    "spine_restored_0_6m",
    "right_arm_parking_restored",
    "pregrasp_entry_pose_verified",
    "restore_counterclockwise_90deg",
    "base_backward_55cm",
    "base_right_140cm",
    "search_right_until_target_or_150cm",
    "rear_wall_baseline_restored",
    "pregrasp_pose_reverified",
    "robust_black_base_pose_alignment_confirmed",
    "grasp_transport_place_and_left_restore",
)


def write_record(path: Path, record: dict) -> None:
    atomic_write_json(path, record)


def execute(record_path: Path) -> int:
    record = {
        "schema_version": 1,
        "status": "running",
        "started_at_unix_s": time.time(),
        "entry_condition": (
            "preceding clockwise turn complete; left arm at calibrated pregrasp; "
            "gripper open"
        ),
        "first_base_motion": "counterclockwise_90deg",
        "parameters": {
            "restore_ccw_deg": RESTORE_CCW_DEG,
            "backward_m": BACKWARD_M,
            "presearch_right_m": PRESEARCH_RIGHT_M,
            "maximum_additional_search_right_m": MAXIMUM_SEARCH_RIGHT_M,
            "rear_clearance_m": REAR_CLEARANCE_M,
            "rear_wall_angle_error_deg": REAR_WALL_ANGLE_DEG,
        },
        "ordered_stages": list(ROUTE_STAGES),
        "completed_stages": [],
        "stage_results": {},
        "physical_motion_authorized": True,
        "right_arm_commanded": True,
    }
    write_record(record_path, record)

    def complete(label: str, result: object) -> None:
        record["stage_results"][label] = result
        record["completed_stages"].append(label)
        record.pop("active_stage", None)
        write_record(record_path, record)

    def active(label: str) -> None:
        record["active_stage"] = label
        write_record(record_path, record)
        print(json.dumps({"stage": label, "status": "starting"}), flush=True)

    code = 2
    try:
        active(ROUTE_STAGES[0])
        complete(ROUTE_STAGES[0], run_stage(
            ROUTE_STAGES[0],
            python_command("quick_start.py", "--parent-lock-held", "--check-only"),
            30.0,
        ))

        active(ROUTE_STAGES[1])
        complete(ROUTE_STAGES[1], run_stage(
            ROUTE_STAGES[1],
            python_command("reset_spine_to_task_height.py", "--execute"),
            620.0,
        ))

        active(ROUTE_STAGES[2])
        complete(ROUTE_STAGES[2], run_stage(
            ROUTE_STAGES[2], python_command("restore_right_parking_direct.py"), 150.0
        ))

        active(ROUTE_STAGES[3])
        complete(ROUTE_STAGES[3], run_stage(
            ROUTE_STAGES[3], python_command("verify_pregrasp_ready.py"), 30.0
        ))

        active(ROUTE_STAGES[4])
        complete(ROUTE_STAGES[4], run_base_controller([
            "--mode", "rotate",
            "--ccw-deg", f"{RESTORE_CCW_DEG:.3f}",
            "--yaw-speed-rps", "0.200",
            "--timeout-s", "45",
        ], 50.0))

        active(ROUTE_STAGES[5])
        complete(ROUTE_STAGES[5], guarded_move_forward_continuous(
            -BACKWARD_M, speed_mps=0.04, timeout_s=35.0
        ))

        active(ROUTE_STAGES[6])
        complete(ROUTE_STAGES[6], guarded_move_right_continuous(
            PRESEARCH_RIGHT_M, speed_mps=0.04, timeout_s=55.0
        ))

        active(ROUTE_STAGES[7])
        search_args = Namespace(
            initial_left_m=0.0,
            maximum_right_m=MAXIMUM_SEARCH_RIGHT_M,
            coarse_step_m=0.08,
            minimum_confidence=0.72,
            center_tolerance_px=20.0,
            reference=DEFAULT_REFERENCE,
            record=DEFAULT_SEARCH_RECORD,
            use_existing_reference=True,
            stop_on_target_visible=True,
        )
        search_code = execute_target_search(search_args)
        if search_code:
            raise RuntimeError(
                f"right target search failed with exit code {search_code}"
            )
        complete(ROUTE_STAGES[7], json.loads(
            DEFAULT_SEARCH_RECORD.read_text(encoding="utf-8")
        ))

        active(ROUTE_STAGES[8])
        complete(ROUTE_STAGES[8], run_base_controller([
            "--mode", "wall-align",
            "--wall-clearance-m", f"{REAR_CLEARANCE_M:.9f}",
            "--wall-angle-deg", f"{REAR_WALL_ANGLE_DEG:.6f}",
            "--clearance-tolerance-m", "0.004",
            "--angle-tolerance-deg", "0.18",
            "--speed-mps", "0.025",
            "--yaw-speed-rps", "0.080",
            "--timeout-s", "120",
        ], 125.0))

        active(ROUTE_STAGES[9])
        complete(ROUTE_STAGES[9], run_stage(
            ROUTE_STAGES[9], python_command("verify_pregrasp_ready.py"), 30.0
        ))

        active(ROUTE_STAGES[10])
        complete(ROUTE_STAGES[10], run_stage(
            ROUTE_STAGES[10],
            python_command("black_base_pose_alignment.py", "--execute"),
            360.0,
        ))

        active(ROUTE_STAGES[11])
        continuation = finish_stage_commands()
        first_grasp_stage = next(
            index for index, (label, _command, _timeout) in enumerate(continuation)
            if label == "force_contact_then_retreat_18mm"
        )
        remaining = continuation[first_grasp_stage:]
        finish_code = execute_finish(
            DEFAULT_FINISH_RECORD,
            stages=remaining,
            entry_condition=(
                "services, spine, pregrasp pose, rear wall and wrist alignment "
                "already confirmed by run_from_ccw_restore_to_finish.py"
            ),
            excluded_stages=[
                "service_preflight_already_complete",
                "spine_reset_already_complete",
                "pregrasp_pose_check_already_complete",
                "pregrasp_lateral_alignment_already_complete",
            ],
        )
        if finish_code:
            raise RuntimeError(
                f"grasp-to-finish pipeline failed with exit code {finish_code}"
            )
        complete(ROUTE_STAGES[11], json.loads(
            DEFAULT_FINISH_RECORD.read_text(encoding="utf-8")
        ))

        record["status"] = "complete"
        code = 0
    except KeyboardInterrupt:
        record["status"] = "interrupted"
        record["error"] = "operator interrupt"
        code = 130
    except Exception as exc:
        record["status"] = "blocked"
        record["error"] = str(exc)
    finally:
        record.pop("active_stage", None)
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
            "entry_condition": (
                "preceding clockwise turn complete; left arm at calibrated pregrasp"
            ),
            "parameters": {
                "restore_ccw_deg": RESTORE_CCW_DEG,
                "backward_m": BACKWARD_M,
                "presearch_right_m": PRESEARCH_RIGHT_M,
                "maximum_additional_search_right_m": MAXIMUM_SEARCH_RIGHT_M,
            },
            "ordered_stages": ROUTE_STAGES,
        }, indent=2))
        return 0

    lock = None
    try:
        lock = acquire_motion_lock()
        return execute(args.record)
    except MissionAlreadyRunning as exc:
        print(json.dumps({"status": "blocked", "error": str(exc)}, indent=2))
        return 73
    finally:
        release_motion_lock(lock)


if __name__ == "__main__":
    raise SystemExit(main())
