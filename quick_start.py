#!/usr/bin/env python3
"""Fast, fail-closed startup for the already-powered Task 2 robot."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from urllib.request import ProxyHandler, build_opener

from mission_runtime import (
    LOCK_FILE,
    MissionAlreadyRunning,
    acquire_motion_lock,
    atomic_write_json,
    release_motion_lock,
)


ROOT = Path(__file__).resolve().parent
REFERENCE_ROOT = Path("/home/aup/tmr-mobile-manipulation")
ROS_ENV = Path("/home/aup/tmr_env.sh")
READY_RECORD = ROOT / "runtime" / "latest_quick_start.json"
VIEWER_URL = "http://127.0.0.1:18081/status.json"
BASE_HOST = "tmr-user@172.16.0.50"
DIRECT_OPENER = build_opener(ProxyHandler({}))

REQUIRED_SERVICES = {
    "/left/controller_manager/list_hardware_components",
    "/left/controller_manager/list_controllers",
    "/left/controller_manager/set_hardware_component_state",
    "/left/controller_manager/switch_controller",
    "/left_ik/compute_fk",
    "/left_ik/compute_ik",
    "/franka_spine_node/get_position",
}
REQUIRED_ACTIONS = {
    "/left/action_server/error_recovery",
    "/left/action_server/ptp_motion",
    "/left/gripper/robotiq_gripper_controller/gripper_cmd",
}


class StartupBlocked(RuntimeError):
    pass


def run(command: list[str], label: str, timeout_s: float = 60.0) -> dict:
    started = time.monotonic()
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
    output = (completed.stdout or "").strip()
    if completed.returncode:
        raise StartupBlocked(
            f"{label} failed with exit code {completed.returncode}: {output[-1500:]}"
        )
    return {
        "label": label,
        "elapsed_s": round(time.monotonic() - started, 3),
        "output": output,
        "output_tail": output[-1500:],
    }


def ros_cli(arguments: list[str], label: str, timeout_s: float = 12.0) -> dict:
    command = (
        f"source {ROS_ENV}; "
        "export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}; "
        + " ".join(arguments)
    )
    return run(["/bin/bash", "-lc", command], label, timeout_s)


def ros_names(kind: str) -> set[str]:
    arguments = ["ros2", kind, "list"]
    if kind == "service":
        arguments.append("--no-daemon")
    result = ros_cli(arguments, f"list_{kind}")
    return {line.strip() for line in result["output"].splitlines() if line.startswith("/")}


def require_core_graph() -> dict:
    services = ros_names("service")
    actions = ros_names("action")
    missing_services = sorted(REQUIRED_SERVICES - services)
    missing_actions = sorted(REQUIRED_ACTIONS - actions)
    if missing_services or missing_actions:
        raise StartupBlocked(
            "core robot services are not running; use the cold-start helper first: "
            f"missing_services={missing_services}, missing_actions={missing_actions}"
        )
    return {
        "label": "core_robot_graph",
        "required_services": len(REQUIRED_SERVICES),
        "required_actions": len(REQUIRED_ACTIONS),
    }


def fetch_viewer_status() -> dict:
    with DIRECT_OPENER.open(VIEWER_URL, timeout=2.0) as response:
        return json.loads(response.read().decode("utf-8"))


def advancing_cameras(delay_s: float = 1.2) -> dict:
    first = fetch_viewer_status()
    time.sleep(delay_s)
    second = fetch_viewer_status()
    required = ("main", "left")
    failed = [
        name for name in required
        if not second.get("healthy", {}).get(name)
        or int(second.get("sequence", {}).get(name, 0))
        <= int(first.get("sequence", {}).get(name, 0))
    ]
    if failed:
        raise StartupBlocked(f"camera frames are stale or unavailable: {failed}")
    return {
        "label": "camera_freshness",
        "sequences_before": {name: first["sequence"][name] for name in required},
        "sequences_after": {name: second["sequence"][name] for name in required},
        "frame_age_s": {name: second["frame_age_s"][name] for name in required},
    }


def ensure_viewer() -> dict:
    try:
        return advancing_cameras()
    except Exception as first_error:
        viewer = REFERENCE_ROOT / "tools" / "three_camera_mjpeg_viewer.py"
        if not viewer.is_file():
            raise StartupBlocked(f"three-camera viewer is missing: {viewer}") from first_error
        # The viewer owns no camera device; it only bridges existing RGB
        # streams, so replacing a stale instance cannot steal FCI or D405.
        subprocess.run(
            ["screen", "-S", "tmr_dual_rgb_viewer", "-X", "quit"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        command = (
            f"source {ROS_ENV}; exec /usr/bin/python3 -u {viewer} --port 18081"
        )
        started = subprocess.run(
            [
                "screen", "-L", "-Logfile", "/tmp/tmr_dual_rgb_viewer.log",
                "-dmS", "tmr_dual_rgb_viewer", "/bin/bash", "-lc", command,
            ],
            check=False,
        )
        if started.returncode:
            raise StartupBlocked("failed to launch the three-camera viewer") from first_error
        deadline = time.monotonic() + 15.0
        last_error: Exception = first_error
        while time.monotonic() < deadline:
            try:
                result = advancing_cameras()
                result["viewer"] = "restarted"
                return result
            except Exception as exc:
                last_error = exc
                time.sleep(0.5)
        raise StartupBlocked(f"camera viewer did not become fresh: {last_error}")


def ensure_base_runtime() -> dict:
    return run(
        [
            "ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", BASE_HOST,
            "bash /home/tmr-user/tmr_cycle/scripts/19_ensure_navigation_stack.sh",
        ],
        "base_runtime",
        150.0,
    )


def ensure_left_runtime() -> dict:
    return ros_cli(
        ["/usr/bin/python3", "-u", str(ROOT / "bootstrap_left_runtime.py"), "--state-only"],
        "left_runtime",
        60.0,
    )


def prepare() -> dict:
    if not ROS_ENV.is_file():
        raise StartupBlocked(f"robot environment is missing: {ROS_ENV}")
    graph = require_core_graph()
    results = [graph]
    # Base and left arm use separate robot interfaces.  Bringing them to a
    # state-only ready condition in parallel removes cold serial wait without
    # putting network or subprocess work in either real-time control loop.
    with ThreadPoolExecutor(max_workers=2) as executor:
        futures = {
            executor.submit(ensure_base_runtime): "base_runtime",
            executor.submit(ensure_left_runtime): "left_runtime",
        }
        for future in as_completed(futures):
            results.append(future.result())
    results.append(ensure_viewer())
    record = {
        "schema_version": 1,
        "status": "ready",
        "prepared_at_unix_s": time.time(),
        "physical_motion_commanded": False,
        "right_arm_commanded": False,
        "results": results,
    }
    atomic_write_json(READY_RECORD, record)
    return record


def check_only() -> dict:
    return {
        "schema_version": 1,
        "status": "healthy",
        "checked_at_unix_s": time.time(),
        "physical_motion_commanded": False,
        "results": [require_core_graph(), advancing_cameras()],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check-only", action="store_true", help="read-only health check")
    mode.add_argument("--execute", action="store_true", help="prepare, then execute the full cycle")
    parser.add_argument("--initial-right-m", type=float, default=2.0)
    args = parser.parse_args()

    lock = None
    try:
        lock = acquire_motion_lock()
        try:
            record = check_only() if args.check_only else prepare()
            print(json.dumps(record, indent=2), flush=True)
            if not args.execute:
                return 0
            completed = subprocess.run(
                [
                    "/usr/bin/python3", "-u", str(ROOT / "run_full_thermal_pad_cycle.py"),
                    "--execute", "--initial-right-m", str(args.initial_right_m),
                    "--prepared-record", str(READY_RECORD),
                    "--parent-lock-held",
                ],
                cwd=ROOT,
                env=os.environ.copy(),
                check=False,
            )
            return completed.returncode
        except (StartupBlocked, subprocess.TimeoutExpired, OSError, ValueError) as exc:
            print(json.dumps({
                "status": "blocked",
                "error": str(exc),
                "physical_motion_commanded": False,
            }, indent=2), file=sys.stderr)
            return 2
    except MissionAlreadyRunning as exc:
        print(json.dumps({"status": "blocked", "error": str(exc)}), file=sys.stderr)
        return 73
    finally:
        release_motion_lock(lock)


if __name__ == "__main__":
    raise SystemExit(main())
