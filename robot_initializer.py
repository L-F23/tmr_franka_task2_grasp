#!/usr/bin/env python3
"""Fail-closed Task 2 startup reset. No command targets the right arm."""

from __future__ import annotations

import json
from pathlib import Path
import ssl
import subprocess
import time
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = ROOT / "config" / "initial_pose.json"
DEFAULT_RECORD = ROOT / "runtime" / "latest_initial_state.json"
ROS_ENV = Path("/home/aup/tmr_env.sh")
RESTORE_LEFT = Path("/home/aup/tmr-mobile-manipulation/grasp/scripts/restore_left_pick_initial.py")
SPINE_API = "https://172.16.16.10/spine/api"


class InitializationError(RuntimeError):
    pass


def run_ros(command: str, timeout: float = 90.0) -> str:
    wrapped = (
        f"source {ROS_ENV}; "
        "export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}; "
        f"{command}"
    )
    result = subprocess.run(
        ["/bin/bash", "-lc", wrapped], text=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        timeout=timeout, check=False,
    )
    if result.returncode != 0:
        raise InitializationError(
            f"command failed ({result.returncode}): {command}\n{result.stdout.strip()}"
        )
    return result.stdout


def spine_request(endpoint: str, method: str = "GET", data: dict | None = None,
                  timeout: float = 10.0) -> Any:
    request = Request(
        f"{SPINE_API}/{endpoint}",
        data=None if data is None else json.dumps(data).encode(),
        method=method,
        headers={"Content-Type": "application/json"},
    )
    try:
        with urlopen(request, timeout=timeout,
                     context=ssl._create_unverified_context()) as response:
            payload = response.read().decode()
    except (HTTPError, URLError, TimeoutError) as exc:
        raise InitializationError(f"Spine {method} {endpoint} failed: {exc}") from exc
    return json.loads(payload) if payload else None


def open_left_gripper(config: dict) -> dict:
    goal = config["left_gripper"]
    output = run_ros(
        "ros2 action send_goal /left/gripper/robotiq_gripper_controller/gripper_cmd "
        "control_msgs/action/GripperCommand "
        f"'{{command: {{position: {goal['target_position']}, max_effort: {goal['max_effort']}}}}}' --feedback",
        timeout=20.0,
    )
    if "status: SUCCEEDED" not in output and "reached_goal: true" not in output:
        raise InitializationError(f"left gripper did not reach open state\n{output}")
    return {"target_position": goal["target_position"], "verified_open": True}


def move_spine(config: dict) -> dict:
    goal = config["spine"]
    target_mm = int(round(goal["target_position_m"] * 1000))
    start_mm = int(spine_request("position-mm")["position"])
    if spine_request("state") != "SwitchedOn":
        spine_request("spine:switch-on", "POST", {})
    if abs(start_mm - target_mm) > goal["maximum_final_error_m"] * 1000:
        result = spine_request(
            "motion-mm:start", "POST",
            {"position": target_mm,
             "velocity": int(round(goal["velocity_m_s"] * 1000)),
             "acceleration": int(round(goal["acceleration_m_s2"] * 1000)),
             "deceleration": int(round(goal["deceleration_m_s2"] * 1000))},
            timeout=600.0,
        )
        if result != "Finished":
            raise InitializationError(f"unexpected Spine result: {result!r}")
    measured_mm = int(spine_request("position-mm")["position"])
    if abs(measured_mm - target_mm) > goal["maximum_final_error_m"] * 1000:
        raise InitializationError(f"Spine endpoint error: {(measured_mm-target_mm)/1000:.6f} m")
    return {"start_position_m": start_mm / 1000,
            "target_position_m": target_mm / 1000,
            "measured_position_m": measured_mm / 1000}


def restore_left_arm() -> dict:
    if not RESTORE_LEFT.is_file():
        raise InitializationError(f"missing left-arm reset script: {RESTORE_LEFT}")
    output = run_ros(f"python3 {RESTORE_LEFT}", timeout=120.0)
    start = output.find("{")
    if start < 0:
        raise InitializationError(f"left-arm reset returned no JSON\n{output}")
    result = json.loads(output[start:])
    if result.get("status") != "success":
        raise InitializationError(f"left-arm reset failed: {result}")
    return result


def initialize(config_path: Path = DEFAULT_CONFIG,
               record_path: Path = DEFAULT_RECORD) -> dict:
    config = json.loads(config_path.read_text(encoding="utf-8"))
    record = {
        "schema_version": 1,
        "started_at_unix": time.time(),
        "sequence": ["open_left_gripper", "move_spine", "restore_left_arm"],
        "right_arm_commanded": False,
        "left_gripper": open_left_gripper(config),
        "spine": move_spine(config),
        "left_arm": restore_left_arm(),
    }
    record.update(completed_at_unix=time.time(), status="ready")
    record_path.parent.mkdir(parents=True, exist_ok=True)
    record_path.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    return record


if __name__ == "__main__":
    print(json.dumps(initialize(), indent=2))
