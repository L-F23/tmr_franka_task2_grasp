#!/usr/bin/env python3
"""Restore the right FR3 to the recorded raised/retracted parking pose."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import rclpy

from restore_left_initial_direct import DirectRestore


ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = ROOT / "config" / "initial_pose.json"


def parking_parameters(config: dict) -> tuple[list[float], float, float]:
    right = config["right_arm"]
    if right.get("policy") != "restore_recorded_parking_pose":
        raise ValueError("right-arm initialization policy is not a recorded parking pose")
    if right.get("commands_allowed_during_initialization") is not True:
        raise ValueError("right-arm initialization commands are disabled")
    target = list(map(float, right["target_positions_rad"]))
    if len(target) != 7:
        raise ValueError("right-arm parking target must contain seven joints")
    return (
        target,
        float(right["maximum_velocity_rad_s"]),
        float(right["maximum_final_error_rad"]),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    target, speed, maximum_error = parking_parameters(config)

    rclpy.init()
    node = DirectRestore("right")
    try:
        result = node.run(target, speed)
        result.update(
            semantic=config["right_arm"].get("semantic"),
            configured_maximum_final_error_rad=maximum_error,
            right_gripper_commanded=False,
        )
        if result["maximum_joint_error_rad"] > maximum_error:
            raise RuntimeError(
                "right parking endpoint exceeds configured tolerance: "
                f"{result['maximum_joint_error_rad']:.6f} rad"
            )
        print(json.dumps(result, indent=2), flush=True)
        return 0
    except Exception as exc:
        print(json.dumps({"status": "blocked", "error": str(exc)}, indent=2), flush=True)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
