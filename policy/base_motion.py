"""Shared odometry-closed-loop motion helpers for the network-visible base."""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import subprocess


LOCAL_MOVER = Path(__file__).resolve().with_name("guarded_lateral_step.py")


def base_process_environment() -> dict[str, str]:
    """Use the container's ROS overlay while allowing testbed DDS overrides."""
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = os.environ.get("TMR_BASE_ROS_DOMAIN_ID", "0")
    environment["ROS_LOCALHOST_ONLY"] = os.environ.get(
        "TMR_BASE_ROS_LOCALHOST_ONLY", "0"
    )
    environment["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    return environment


def check_base_runtime(timeout_s: float = 15.0) -> dict:
    """Verify the deployed base topics over DDS without starting or moving anything."""
    completed = subprocess.run(
        ["ros2", "topic", "list"], text=True, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, timeout=timeout_s, check=False,
        env=base_process_environment(),
    )
    output = (completed.stdout or "").strip()
    if completed.returncode:
        raise RuntimeError(
            "base DDS graph query failed; the standard testbed base services must "
            f"already be running. exit={completed.returncode}, output={output[-1200:]}"
        )
    topics = {line.strip() for line in output.splitlines() if line.startswith("/")}
    required = {"/swerve_drive_controller/odom", "/tmr_cycle/mission_cmd_vel"}
    missing = sorted(required - topics)
    if missing:
        raise RuntimeError(
            "base topics are not visible from the host-network container; verify "
            "the testbed CycloneDDS/domain configuration. "
            f"missing={missing}"
        )
    return {"label": "base_runtime", "output": output}


def _mover_command(arguments: str) -> list[str]:
    """Run the bundled Task 2 mover locally against the testbed DDS graph."""
    return [
        sys.executable, "-u", str(LOCAL_MOVER),
        *arguments.split(), "--disable-collision-guard",
    ]


def _run_mover(arguments: str, *, timeout_s: float | None = None):
    return subprocess.run(
        _mover_command(arguments),
        check=False,
        text=True,
        capture_output=True,
        timeout=timeout_s,
        env=base_process_environment(),
    )


def _extract_last_json_object(output: str) -> dict:
    decoder = json.JSONDecoder()
    best: tuple[int, int, dict] | None = None
    for index, character in enumerate(output):
        if character != "{":
            continue
        try:
            value, consumed = decoder.raw_decode(output[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            candidate = (index + consumed, -index, value)
            if best is None or candidate[:2] > best[:2]:
                best = candidate
    if best is None:
        raise RuntimeError("base mover returned no JSON result")
    return best[2]


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
    """Execute one remote step with odometry feedback and collision guard disabled."""
    if not 0.008 <= abs(distance_m) <= 0.08:
        raise ValueError("absolute step distance must be in [0.008, 0.08] m")
    completed = _run_mover(
        f"--right-m {distance_m:.6f} --speed-mps {speed_mps:.4f} "
        f"--timeout-s {timeout_s:.1f}",
        timeout_s=timeout_s + 20.0,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"guarded base step failed: {completed.stdout.strip()} "
            f"{completed.stderr.strip()}"
        )
    result = _extract_last_json_object(completed.stdout)
    if result.get("status") != "success":
        raise RuntimeError(f"guarded base step did not succeed: {result}")
    return result


def guarded_move_right_continuous(
    distance_m: float, *, speed_mps: float = 0.04, timeout_s: float = 80.0
) -> dict:
    """Execute one uninterrupted long lateral move with live odometry feedback."""
    if not 0.008 <= abs(distance_m) <= 2.0:
        raise ValueError("absolute continuous distance must be in [0.008, 2.0] m")
    completed = _run_mover(
        f"--right-m {distance_m:.6f} --speed-mps {speed_mps:.4f} "
        f"--timeout-s {timeout_s:.1f}",
        timeout_s=timeout_s + 20.0,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"continuous guarded base move failed: {completed.stdout.strip()} "
            f"{completed.stderr.strip()}"
        )
    result = _extract_last_json_object(completed.stdout)
    if result.get("status") != "success":
        raise RuntimeError(f"continuous guarded base move did not succeed: {result}")
    return result


def guarded_move_forward(
    distance_m: float, *, speed_mps: float = 0.02, timeout_s: float = 15.0
) -> dict:
    """Execute one signed fore/aft step with odometry feedback."""
    if not 0.008 <= abs(distance_m) <= 0.08:
        raise ValueError("absolute step distance must be in [0.008, 0.08] m")
    completed = _run_mover(
        f"--forward-m {distance_m:.6f} --speed-mps {speed_mps:.4f} "
        f"--timeout-s {timeout_s:.1f}",
        timeout_s=timeout_s + 20.0,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"guarded fore/aft step failed: {completed.stdout.strip()} "
            f"{completed.stderr.strip()}"
        )
    result = _extract_last_json_object(completed.stdout)
    if result.get("status") != "success":
        raise RuntimeError(f"guarded fore/aft step did not succeed: {result}")
    return result


def guarded_move_forward_continuous(
    distance_m: float, *, speed_mps: float = 0.04, timeout_s: float = 80.0
) -> dict:
    """Execute one uninterrupted long fore/aft move with odometry feedback."""
    if not 0.008 <= abs(distance_m) <= 2.0:
        raise ValueError("absolute continuous distance must be in [0.008, 2.0] m")
    completed = _run_mover(
        f"--forward-m {distance_m:.6f} --speed-mps {speed_mps:.4f} "
        f"--timeout-s {timeout_s:.1f}",
        timeout_s=timeout_s + 20.0,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"continuous guarded fore/aft move failed: {completed.stdout.strip()} "
            f"{completed.stderr.strip()}"
        )
    result = _extract_last_json_object(completed.stdout)
    if result.get("status") != "success":
        raise RuntimeError(f"continuous fore/aft move did not succeed: {result}")
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
