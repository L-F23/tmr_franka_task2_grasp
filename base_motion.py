"""Shared guarded lateral-base motion helpers for the isolated base host."""

from __future__ import annotations

import json
import subprocess


BASE_HOST = "tmr-user@172.16.0.50"
REMOTE_MOVER = "/home/tmr-user/tmr_cycle/scripts/guarded_lateral_step.py"
BASE_ENV = (
    "source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
    "source /home/tmr-user/ros2_ws/install/setup.bash >/dev/null 2>&1 || true; "
    "export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 "
    "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp "
    "CYCLONEDDS_URI=file:///home/tmr-user/cyclonedds.xml"
)


def split_lateral_move(distance_m: float, maximum_step_m: float = 0.08) -> list[float]:
    """Split a signed displacement into commands accepted by the guarded mover."""
    if not 0.008 <= maximum_step_m <= 0.08:
        raise ValueError("maximum_step_m must be in [0.008, 0.08]")
    if abs(distance_m) < 1e-9:
        return []
    direction = 1.0 if distance_m > 0.0 else -1.0
    remaining = abs(float(distance_m))
    steps = []
    while remaining > maximum_step_m:
        steps.append(direction * maximum_step_m)
        remaining -= maximum_step_m
    if remaining >= 0.008:
        steps.append(direction * remaining)
    elif steps:
        steps[-1] += direction * remaining
    else:
        steps.append(direction * 0.008)
    return steps


def guarded_move_right(
    distance_m: float, *, speed_mps: float = 0.02, timeout_s: float = 15.0
) -> dict:
    """Execute one remote step; the remote process enforces odometry and two LiDARs."""
    if not 0.008 <= abs(distance_m) <= 0.08:
        raise ValueError("absolute step distance must be in [0.008, 0.08] m")
    command = (
        f"{BASE_ENV}; python3 {REMOTE_MOVER} --right-m {distance_m:.6f} "
        f"--speed-mps {speed_mps:.4f} --timeout-s {timeout_s:.1f}"
    )
    completed = subprocess.run(
        ["ssh", "-o", "BatchMode=yes", BASE_HOST, command],
        check=False,
        text=True,
        capture_output=True,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"guarded base step failed: {completed.stdout.strip()} "
            f"{completed.stderr.strip()}"
        )
    start = completed.stdout.find("{")
    if start < 0:
        raise RuntimeError("guarded base step returned no JSON result")
    result = json.loads(completed.stdout[start:])
    if result.get("status") != "success":
        raise RuntimeError(f"guarded base step did not succeed: {result}")
    return result


def guarded_move_right_continuous(
    distance_m: float, *, speed_mps: float = 0.04, timeout_s: float = 80.0
) -> dict:
    """Execute one uninterrupted long lateral move with live odom/LiDAR guards."""
    if not 0.008 <= abs(distance_m) <= 2.0:
        raise ValueError("absolute continuous distance must be in [0.008, 2.0] m")
    command = (
        f"{BASE_ENV}; python3 {REMOTE_MOVER} --right-m {distance_m:.6f} "
        f"--speed-mps {speed_mps:.4f} --timeout-s {timeout_s:.1f}"
    )
    completed = subprocess.run(
        ["ssh", "-o", "BatchMode=yes", BASE_HOST, command],
        check=False,
        text=True,
        capture_output=True,
        timeout=timeout_s + 20.0,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"continuous guarded base move failed: {completed.stdout.strip()} "
            f"{completed.stderr.strip()}"
        )
    start = completed.stdout.find("{")
    if start < 0:
        raise RuntimeError("continuous guarded base move returned no JSON result")
    result = json.loads(completed.stdout[start:])
    if result.get("status") != "success":
        raise RuntimeError(f"continuous guarded base move did not succeed: {result}")
    return result


def guarded_move_forward(
    distance_m: float, *, speed_mps: float = 0.02, timeout_s: float = 15.0
) -> dict:
    """Execute one signed fore/aft step with odometry and dual-LiDAR guards."""
    if not 0.008 <= abs(distance_m) <= 0.08:
        raise ValueError("absolute step distance must be in [0.008, 0.08] m")
    command = (
        f"{BASE_ENV}; python3 {REMOTE_MOVER} --forward-m {distance_m:.6f} "
        f"--speed-mps {speed_mps:.4f} --timeout-s {timeout_s:.1f}"
    )
    completed = subprocess.run(
        ["ssh", "-o", "BatchMode=yes", BASE_HOST, command],
        check=False,
        text=True,
        capture_output=True,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"guarded fore/aft step failed: {completed.stdout.strip()} "
            f"{completed.stderr.strip()}"
        )
    start = completed.stdout.find("{")
    if start < 0:
        raise RuntimeError("guarded fore/aft step returned no JSON result")
    result = json.loads(completed.stdout[start:])
    if result.get("status") != "success":
        raise RuntimeError(f"guarded fore/aft step did not succeed: {result}")
    return result


def guarded_transport(
    distance_m: float,
    *,
    tolerance_m: float = 0.005,
    maximum_steps: int = 40,
    mover=guarded_move_right,
) -> tuple[list[dict], float]:
    """Close the loop on accumulated odometry instead of accumulating step error."""
    if abs(distance_m) < 0.008:
        raise ValueError("transport distance must be at least 0.008 m")
    completed_m = 0.0
    results = []
    for index in range(1, maximum_steps + 1):
        remaining_m = float(distance_m) - completed_m
        if abs(remaining_m) <= tolerance_m:
            return results, completed_m
        command_m = (1.0 if remaining_m > 0.0 else -1.0) * min(
            0.08, max(0.008, abs(remaining_m))
        )
        result = mover(command_m)
        actual_m = float(result["actual_right_m"])
        if actual_m * remaining_m <= 0.0 or abs(actual_m) < 0.001:
            raise RuntimeError(
                f"guarded transport made no progress: remaining={remaining_m:.6f}, "
                f"actual={actual_m:.6f}"
            )
        item = dict(result)
        item.update(index=index, cumulative_right_m=completed_m + actual_m)
        results.append(item)
        completed_m += actual_m
    raise RuntimeError(
        f"guarded transport did not converge after {maximum_steps} steps: "
        f"requested={distance_m:.6f}, actual={completed_m:.6f}"
    )
