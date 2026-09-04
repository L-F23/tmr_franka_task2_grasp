import pytest

from run_from_pregrasp_to_finish import STAGE_ORDER, stage_commands
from verify_pregrasp_ready import pose_errors, validate_errors


def flattened_commands():
    return [argument for _label, command, _timeout in stage_commands() for argument in command]


def test_runner_starts_with_read_only_gates_before_motion():
    stages = stage_commands()
    assert stages[0][1][-1] == "--check-only"
    assert stages[1][1][2].endswith("verify_pregrasp_ready.py")
    assert "--execute" not in stages[0][1]
    assert "--execute" not in stages[1][1]


def test_runner_excludes_transport_search_and_pregrasp_motion():
    commands = flattened_commands()
    assert "run_full_thermal_pad_cycle.py" not in commands
    assert "align_to_thermal_pad.py" not in commands
    assert "table_edge_positioning.py" not in commands
    assert "execute_thermal_pad_grasp.py" not in commands
    assert "2.0" not in commands


def test_runner_requires_alignment_before_gripper_close():
    aligned = STAGE_ORDER.index("pregrasp_lateral_alignment_confirmed")
    grasped = STAGE_ORDER.index("thermal_pad_grasped")
    assert aligned + 1 == grasped


def test_red_station_uses_wrist_closed_loop():
    red_command = next(
        command for label, command, _timeout in stage_commands()
        if label == "red_pad_centered_under_left_wrist"
    )
    assert "--wrist-closed-loop" in red_command


def test_pose_gate_accepts_exact_reference_and_rejects_offset():
    reference = {
        "expected_joint_positions_rad": [0.0] * 7,
        "expected_link8_base_position_m": [0.8, 0.0, 0.8],
        "expected_link8_base_orientation_xyzw": [0.5, -0.5, 0.5, -0.5],
        "maximum_joint_error_rad": 0.035,
        "maximum_position_error_m": 0.02,
        "maximum_orientation_error_deg": 5.0,
    }
    measured = {
        "joint_positions_rad": [0.0] * 7,
        "link8_base_position_m": [0.8, 0.0, 0.8],
        "link8_base_orientation_xyzw": [0.5, -0.5, 0.5, -0.5],
    }
    validate_errors(pose_errors(measured, reference), reference)
    measured["link8_base_position_m"] = [0.83, 0.0, 0.8]
    with pytest.raises(RuntimeError, match="pregrasp pose"):
        validate_errors(pose_errors(measured, reference), reference)
