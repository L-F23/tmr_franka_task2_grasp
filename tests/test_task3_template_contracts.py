import json
from pathlib import Path

import base_motion
import quick_start
import run_from_pregrasp_to_finish
from mission_runtime import LOCK_FILE, atomic_write_json


ROOT = Path(__file__).resolve().parents[1] / "policy"


def test_all_motion_entrypoints_share_one_task_lock():
    assert quick_start.LOCK_FILE == LOCK_FILE
    assert run_from_pregrasp_to_finish.LOCK_FILE == LOCK_FILE


def test_base_mover_runs_staged_source_in_isolated_base_graph():
    command = base_motion._mover_command("--right-m 0.020")
    joined = " ".join(command)
    assert "guarded_lateral_step.py --right-m 0.020 --disable-collision-guard" in joined
    assert "tmr-mobile-manipulation" not in joined
    assert command[0] == "ssh"
    assert "ROS_DOMAIN_ID=${TMR_CYCLE_ROS_DOMAIN_ID:-97}" in joined
    assert "ROS_LOCALHOST_ONLY=${TMR_CYCLE_ROS_LOCALHOST_ONLY:-1}" in joined


def test_base_login_does_not_assume_a_host_ssh_directory():
    command = base_motion.ssh_command("true")
    joined = " ".join(command)
    assert "/home/aup/.ssh" not in joined
    assert "UserKnownHostsFile=/tmp/tmr_task2_known_hosts" in joined


def test_arm_collision_gate_is_explicitly_disabled():
    config = json.loads((ROOT / "config" / "thermal_pad_pick.json").read_text())
    assert config["kinematics"]["avoid_collisions"] is False
    source = (ROOT / "thermal_pad_ik.py").read_text()
    assert "direct_joint_interpolation_collision_guard_disabled" in source


def test_initial_restore_requires_action_and_target_success():
    source = (ROOT / "restore_left_initial_direct.py").read_text()
    assert "wrapped.status != GoalStatus.STATUS_SUCCEEDED" in source
    assert "target_status != wrapped.result.target_status.TARGET_REACHED" in source


def test_atomic_checkpoint_replaces_complete_json(tmp_path):
    path = tmp_path / "state.json"
    atomic_write_json(path, {"phase": "ONE"})
    atomic_write_json(path, {"phase": "TWO", "done": True})
    assert json.loads(path.read_text()) == {"phase": "TWO", "done": True}


def test_base_report_parser_ignores_logs_and_uses_last_complete_object():
    output = 'log {not-json}\n{"status":"old"}\nnoise\n{"status":"success","x":1}\n'
    assert base_motion._extract_last_json_object(output) == {
        "status": "success",
        "x": 1,
    }
