#!/usr/bin/env python3
"""Reset Spine to the canonical Task 2 height without moving either arm."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from mission_runtime import atomic_write_json
from robot_initializer import DEFAULT_CONFIG, move_spine


ROOT = Path(__file__).resolve().parent
DEFAULT_RECORD = ROOT / "config" / "latest_spine_reset.json"


def reset_spine(config_path: Path, record_path: Path) -> dict:
    config = json.loads(config_path.read_text(encoding="utf-8"))
    goal = config["spine"]
    target_m = float(goal["target_position_m"])
    if abs(target_m - 0.6) > 1e-9:
        raise ValueError(
            f"Task 2 canonical Spine target must be 0.600 m, got {target_m:.6f} m"
        )
    record = {
        "schema_version": 1,
        "status": "starting",
        "started_at_unix_s": time.time(),
        "config": str(config_path),
        "target_position_m": target_m,
        "spine_commanded": True,
        "left_arm_commanded": False,
        "right_arm_commanded": False,
        "gripper_commanded": False,
    }
    atomic_write_json(record_path, record)
    try:
        record["spine"] = move_spine(config)
        record["status"] = "spine_task_height_restored"
        return record
    except Exception as exc:
        record["status"] = "blocked"
        record["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        record["finished_at_unix_s"] = time.time()
        atomic_write_json(record_path, record)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--record", type=Path, default=DEFAULT_RECORD)
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required for Spine motion")
    try:
        report = reset_spine(args.config, args.record)
        print(json.dumps(report, indent=2), flush=True)
        return 0
    except Exception as exc:
        print(json.dumps({
            "status": "blocked",
            "error": f"{type(exc).__name__}: {exc}",
        }, indent=2), flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
