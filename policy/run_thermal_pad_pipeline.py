#!/usr/bin/env python3
"""Ordered reset -> base alignment -> thermal-pad FK/IK validation pipeline."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import time


ROOT = Path(__file__).resolve().parent
RESTORE = ROOT / "restore_left_initial_direct.py"


def run(command: list[str], label: str, timeout: float) -> dict:
    completed = subprocess.run(
        command,
        cwd=ROOT,
        env=os.environ.copy(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        check=False,
    )
    print(completed.stdout, end="" if completed.stdout.endswith("\n") else "\n", flush=True)
    if completed.returncode:
        raise RuntimeError(f"{label} failed with exit code {completed.returncode}")
    return {"label": label, "returncode": completed.returncode}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="authorize left-arm reset and low-speed base alignment; IK remains non-actuating",
    )
    parser.add_argument(
        "--allow-odom-only",
        action="store_true",
        help="explicitly allow base alignment without the dual-lidar gate",
    )
    parser.add_argument("--maximum-alignment-steps", type=int, default=12)
    args = parser.parse_args()
    record = {
        "started_at_unix": time.time(),
        "execute_authorized": bool(args.execute),
        "ordered_stages": [],
        "right_arm_commanded": False,
        "gripper_commanded": False,
    }
    try:
        if args.execute:
            run(
                ["/usr/bin/python3", "-u", str(ROOT / "bootstrap_left_runtime.py"), "--state-only"],
                "bootstrap_left_runtime",
                45.0,
            )
            record["ordered_stages"].append("left_runtime_ready")
            run(["/usr/bin/python3", "-u", str(RESTORE)], "restore_left_initial", 150.0)
            record["ordered_stages"].append("left_initial_verified")
        else:
            record["ordered_stages"].append("left_reset_skipped_dry_run")

        alignment = [
            "/usr/bin/python3", "-u", str(ROOT / "align_to_thermal_pad.py"),
            "--maximum-steps", str(args.maximum_alignment_steps),
        ]
        if args.execute:
            alignment.append("--execute")
        if args.allow_odom_only:
            alignment.append("--allow-odom-only")
        run(alignment, "base_visual_alignment", 180.0)
        record["ordered_stages"].append("thermal_pad_centered")

        # Odom must prove a full stationary interval in the planner itself.
        time.sleep(1.2)
        run(
            ["/usr/bin/python3", "-u", str(ROOT / "thermal_pad_ik.py")],
            "thermal_pad_fk_ik",
            90.0,
        )
        record["ordered_stages"].append("fk_ik_validation_complete")
        record["status"] = "complete"
        return_code = 0
    except Exception as exc:
        record.update(status="blocked", error=str(exc))
        return_code = 2
    record["completed_at_unix"] = time.time()
    output = ROOT / "config" / "latest_thermal_pad_pipeline.json"
    output.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(record, indent=2), flush=True)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
