import json
from pathlib import Path

import base_motion
import quick_start
import run_from_pregrasp_to_finish
from mission_runtime import LOCK_FILE, atomic_write_json


ROOT = Path(__file__).resolve().parents[1]


def test_all_motion_entrypoints_share_one_task_lock():
    assert quick_start.LOCK_FILE == LOCK_FILE
    assert run_from_pregrasp_to_finish.LOCK_FILE == LOCK_FILE


def test_base_mover_is_task2_source_stream_with_collision_guard_disabled():
    command, source = base_motion._remote_mover_command("--right-m 0.020")
    joined = " ".join(command)
    assert "python3 - --right-m 0.020 --disable-collision-guard" in joined
    assert "timeout --signal=INT --kill-after=3" in joined
    assert "tmr-mobile-manipulation" not in joined
    assert "class GuardedLateralStep" in source


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
