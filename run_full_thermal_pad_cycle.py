#!/usr/bin/env python3
"""Run the odometry-closed-loop 2 m approach and thermal-pad transfer cycle."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import time

from base_motion import guarded_move_right_continuous
from mission_runtime import (
    MissionAlreadyRunning,
    acquire_motion_lock,
    atomic_write_json,
    release_motion_lock,
)


ROOT = Path(__file__).resolve().parent
DEFAULT_RECORD = ROOT / "config" / "latest_full_thermal_pad_cycle.json"
MAX_PREPARED_RECORD_AGE_S = 20.0
FULL_STAGE_ORDER = (
    "left_runtime_ready",
    "left_initial_verified_before_base_transport",
    "isolated_base_runtime_ready",
    "base_right_2m_complete",
    "black_base_and_thermal_pad_centered",
    "table_edge_fore_aft_aligned",
    "left_pregrasp_reached",
    "pregrasp_lateral_alignment_confirmed",
    "left_grasp_pose_reached",
    "thermal_pad_grasped",
    "left_arm_lifted_12cm",
    "red_pad_station_reached",
    "placement_forward_143mm_down_12cm_complete",
    "placement_retract_tilt_with_gripper_held",
    "gripper_open_then_vertical_clearance_5cm",
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


def validate_prepared_record(path: Path, now: float | None = None) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    age = (time.time() if now is None else now) - float(value["prepared_at_unix_s"])
    if value.get("schema_version") != 1 or value.get("status") != "ready":
        raise RuntimeError("quick-start prepared record is not ready")
    if age < 0.0 or age > MAX_PREPARED_RECORD_AGE_S:
        raise RuntimeError(f"quick-start prepared record is stale: {age:.3f}s")
    if value.get("physical_motion_commanded") is not False:
        raise RuntimeError("invalid quick-start prepared record motion flag")
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--initial-right-m", type=float, default=2.0)
    parser.add_argument("--maximum-alignment-steps", type=int, default=20)
    parser.add_argument("--record", type=Path, default=DEFAULT_RECORD)
    parser.add_argument("--prepared-record", type=Path)
    parser.add_argument("--parent-lock-held", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument(
        "--resume-after-transport", action="store_true",
        help="resume at visual alignment after a completed initial base transport",
    )
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

    lock = None
    if not args.parent_lock_held:
        try:
            lock = acquire_motion_lock()
        except MissionAlreadyRunning as exc:
            print(json.dumps({"status": "blocked", "error": str(exc)}, indent=2))
            return 73

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
        "active_stage": None,
    }
    atomic_write_json(args.record, record)
    code = 2
    try:
        record["active_stage"] = FULL_STAGE_ORDER[0]
        atomic_write_json(args.record, record)
        if args.prepared_record is None:
            record["stage_results"].append(run(
                python_command("bootstrap_left_runtime.py", "--state-only"),
                "bootstrap_left_runtime",
                60.0,
            ))
        else:
            prepared = validate_prepared_record(args.prepared_record)
            record["stage_results"].append({
                "label": "quick_start_prepared_runtime",
                "prepared_at_unix_s": prepared["prepared_at_unix_s"],
            })
        record["ordered_stages"].append(FULL_STAGE_ORDER[0])
        record["active_stage"] = FULL_STAGE_ORDER[1]
        atomic_write_json(args.record, record)
        record["stage_results"].append(run(
            python_command("restore_left_initial_direct.py"),
            "restore_left_initial_before_base_transport",
            150.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[1])
        record["active_stage"] = FULL_STAGE_ORDER[2]
        atomic_write_json(args.record, record)

        if args.prepared_record is None:
            record["stage_results"].append(run(
                [
                    "ssh", "-o", "BatchMode=yes", "tmr-user@172.16.0.50",
                    "bash /home/tmr-user/tmr_cycle/scripts/19_ensure_navigation_stack.sh",
                ],
                "ensure_isolated_base_runtime",
                150.0,
            ))
        else:
            # The prepared record is consumed quickly and validated above;
            # the base mover still checks fresh odometry and the exclusive
            # mission command subscriber before every physical step.
            record["stage_results"].append({"label": "quick_start_base_runtime_reused"})
        record["ordered_stages"].append(FULL_STAGE_ORDER[2])
        record["active_stage"] = FULL_STAGE_ORDER[3]
        atomic_write_json(args.record, record)

        if args.resume_after_transport:
            transport_result = {"status": "reused", "motion_commanded": False}
            record["base_transport_steps"].append(transport_result)
            record["actual_initial_right_m"] = None
        else:
            transport_result = guarded_move_right_continuous(args.initial_right_m)
            record["base_transport_steps"].append(transport_result)
            print(json.dumps({"continuous_base_transport": transport_result}), flush=True)
            record["actual_initial_right_m"] = float(transport_result["actual_right_m"])
        record["ordered_stages"].append(FULL_STAGE_ORDER[3])
        record["active_stage"] = FULL_STAGE_ORDER[4]
        atomic_write_json(args.record, record)

        record["stage_results"].append(run(
            python_command(
                "align_to_thermal_pad.py",
                "--execute",
                "--maximum-steps", str(args.maximum_alignment_steps),
            ),
            "black_base_visual_alignment",
            420.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[4])
        record["active_stage"] = FULL_STAGE_ORDER[5]
        atomic_write_json(args.record, record)

        record["stage_results"].append({
            "label": "table_edge_fore_aft_alignment",
            "status": "skipped",
            "reason": "pregrasp base fore/aft motion disabled by operator",
            "physical_motion_commanded": False,
        })
        record["ordered_stages"].append(FULL_STAGE_ORDER[5])
        record["active_stage"] = FULL_STAGE_ORDER[6]
        atomic_write_json(args.record, record)

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
        record["ordered_stages"].append(FULL_STAGE_ORDER[6])
        record["active_stage"] = FULL_STAGE_ORDER[7]
        atomic_write_json(args.record, record)

        record["stage_results"].append(run(
            python_command("black_base_pose_alignment.py", "--execute"),
            "mandatory_pregrasp_black_base_pose_alignment",
            360.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[7])
        record["active_stage"] = FULL_STAGE_ORDER[8]
        atomic_write_json(args.record, record)

        record["stage_results"].append(run(
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
            "left_grasp_pose",
            180.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[8])
        record["active_stage"] = FULL_STAGE_ORDER[9]
        atomic_write_json(args.record, record)

        record["stage_results"].append(run(
            python_command("stage1_close_gripper.py", "--execute"),
            "thermal_pad_grasp",
            60.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[9])
        record["active_stage"] = FULL_STAGE_ORDER[10]
        atomic_write_json(args.record, record)

        record["stage_results"].append(run(
            python_command(
                "set_stage1_start_from_current.py",
                "--execute", "--backward-m", "0", "--forward-m", "0",
                "--down-m", "0", "--up-m", "0.12",
                "--speed-rad-s", "0.05", "--fast",
                "--record", str(ROOT / "config" / "latest_stage2_lift.json"),
            ),
            "left_vertical_lift",
            90.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[10])
        record["active_stage"] = FULL_STAGE_ORDER[11]
        atomic_write_json(args.record, record)

        record["stage_results"].append(run(
            python_command(
                "stage3_red_pad_alignment.py",
                "--execute", "--from-black-base-reference",
            ),
            "red_pad_station_alignment",
            180.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[11])
        record["active_stage"] = FULL_STAGE_ORDER[12]
        atomic_write_json(args.record, record)

        record["stage_results"].append(run(
            python_command(
                "set_stage1_start_from_current.py",
                "--execute", "--backward-m", "0", "--forward-m", "0.143",
                "--down-m", "0.12", "--up-m", "0",
                "--speed-rad-s", "0.05", "--fast",
                "--record", str(ROOT / "config" / "latest_stage4_place_approach.json"),
            ),
            "placement_approach",
            150.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[12])
        record["active_stage"] = FULL_STAGE_ORDER[13]
        atomic_write_json(args.record, record)

        record["stage_results"].append(run(
            python_command(
                "stage5_release_diagonal.py", "--execute",
                "--backward-m", "0.10", "--initial-down-m", "0.015",
                "--down-m", "0.035", "--pre-open-contact-drop-m", "0.015",
                "--maximum-contact-drop-m", "0.05",
                "--tilt-down-deg", "20", "--open-after-m", "0.08",
                "--open-travel-fraction", "0.4",
                "--defer-gripper-open",
                "--terminal-left-correction-m", "0.012",
                "--final-lift-m", "-0.005", "--final-extra-tilt-deg", "25",
                "--follow-through-inward-m", "0.020",
                "--follow-through-extra-tilt-deg", "10",
                "--follow-through-contact-z-delta-m", "0",
                "--speed-rad-s", "0.05",
            ),
            "retract_down_then_release",
            120.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[13])
        record["active_stage"] = FULL_STAGE_ORDER[14]
        atomic_write_json(args.record, record)

        record["stage_results"].append(run(
            python_command(
                "set_stage1_start_from_current.py",
                "--execute", "--backward-m", "0", "--forward-m", "0",
                "--down-m", "0", "--up-m", "0.05",
                "--speed-rad-s", "0.05", "--fast",
                "--open-gripper-before-motion",
                "--record", str(ROOT / "config" / "latest_post_release_clearance.json"),
            ),
            "post_release_vertical_clearance",
            90.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[14])
        record["active_stage"] = FULL_STAGE_ORDER[15]
        atomic_write_json(args.record, record)

        record["stage_results"].append(run(
            python_command("restore_left_initial_direct.py"),
            "restore_left_initial_after_release",
            150.0,
        ))
        record["ordered_stages"].append(FULL_STAGE_ORDER[15])
        record.pop("active_stage", None)
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
        record["completed_at_unix_s"] = time.time()
        atomic_write_json(args.record, record)
        print(json.dumps(record, indent=2), flush=True)
        release_motion_lock(lock)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
