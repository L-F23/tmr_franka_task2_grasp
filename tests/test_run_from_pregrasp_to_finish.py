import pytest

from run_from_pregrasp_to_finish import (
    FAST_ARM_SPEED_RAD_S,
    FAST_RELEASE_SPEED_RAD_S,
    STAGE_ORDER,
    stage_commands,
)
from verify_pregrasp_ready import pose_errors, validate_errors


def flattened_commands():
    return [argument for _label, command, _timeout in stage_commands() for argument in command]


def test_runner_checks_health_then_resets_spine_before_pose_gate():
    stages = stage_commands()
    assert stages[0][1][-1] == "--check-only"
    assert stages[1][1][2].endswith("reset_spine_to_task_height.py")
    assert "--execute" in stages[1][1]
    assert stages[2][1][2].endswith("restore_right_parking_direct.py")
    assert stages[3][1][2].endswith("verify_pregrasp_ready.py")
    assert "--execute" not in stages[0][1]
    assert "--execute" not in stages[3][1]
    assert "--parent-lock-held" in stages[0][1]


def test_runner_excludes_transport_search_and_pregrasp_motion():
    commands = flattened_commands()
    assert "run_full_thermal_pad_cycle.py" not in commands
    assert "align_to_thermal_pad.py" not in commands
    assert "table_edge_positioning.py" not in commands
    assert "execute_thermal_pad_grasp.py" not in commands
    assert not any(
        argument in commands
        for argument in ("--base-right-m", "--right-m", "--left-m")
    )


def test_runner_parks_right_arm_before_pregrasp_verification():
    labels = [label for label, _command, _timeout in stage_commands()]
    assert labels.index("right_arm_parking_restored") < labels.index(
        "calibrated_pregrasp_pose_verified"
    )


def test_runner_enforces_pregrasp_alignment_and_close_gates():
    commands = stage_commands()
    assert "pregrasp_lateral_alignment_confirmed" in STAGE_ORDER
    assert any(
        command[2].endswith("black_base_pose_alignment.py")
        for _label, command, _timeout in commands
    )
    close_command = next(
        command for label, command, _timeout in commands
        if label == "thermal_pad_grasped"
    )
    assert "--skip-pregrasp-calibration-gates" not in close_command


def test_red_station_uses_main_camera_with_wrist_advisory_only():
    red_command = next(
        command for label, command, _timeout in stage_commands()
        if label == "red_pad_station_reached_main_camera_wrist_advisory"
    )
    assert "--wrist-closed-loop" not in red_command


def test_fast_profile_is_applied_to_every_arm_translation_and_release():
    commands = stage_commands()
    arm_motion_labels = {
        "left_arm_lifted_12cm",
        "placement_forward_143mm_down_12cm_complete",
        "gripper_open_then_vertical_clearance_5cm",
    }
    for label, command, _timeout in commands:
        if label in arm_motion_labels:
            speed_index = command.index("--speed-rad-s") + 1
            assert command[speed_index] == FAST_ARM_SPEED_RAD_S
    release = next(
        command for label, command, _timeout in commands
        if label == "placement_retract_tilt_with_gripper_held"
    )
    assert release[release.index("--speed-rad-s") + 1] == FAST_RELEASE_SPEED_RAD_S
    assert "--defer-gripper-open" in release
    assert release[release.index("--terminal-left-correction-m") + 1] == "0.012"
    clearance = next(
        command for label, command, _timeout in commands
        if label == "gripper_open_then_vertical_clearance_5cm"
    )
    assert "--open-gripper-before-motion" in clearance


def test_grasp_approach_opens_and_uses_incremental_external_contact_guard():
    command = next(
        command for label, command, _timeout in stage_commands()
        if label == "force_contact_then_retreat_18mm"
    )
    assert command[command.index("--forward-m") + 1] == "0.162"
    assert command[command.index("--speed-rad-s") + 1] == "0.025"
    assert "--guarded-contact-approach" in command
    assert command[command.index("--contact-step-m") + 1] == "0.002"
    assert command[command.index("--contact-retreat-m") + 1] == "0.018"
    assert command[command.index("--axis-force-delta-n") + 1] == "2.5"
    assert command[command.index("--torque-delta-norm-nm") + 1] == "1.5"
    assert command[command.index("--joint-torque-delta-nm") + 1] == "2.0"
    assert command[command.index("--contact-consecutive-samples") + 1] == "5"


def test_lift_has_no_forward_motion_and_place_keeps_forward_approach():
    commands = {label: command for label, command, _timeout in stage_commands()}
    lift = commands["left_arm_lifted_12cm"]
    place = commands["placement_forward_143mm_down_12cm_complete"]
    assert lift[lift.index("--forward-m") + 1] == "0"
    assert lift[lift.index("--up-m") + 1] == "0.12"
    assert place[place.index("--forward-m") + 1] == "0.143"
    assert place[place.index("--down-m") + 1] == "0.12"


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


def test_pose_gate_accepts_redundant_joint_solution_when_task_pose_matches():
    reference = {
        "expected_joint_positions_rad": [0.0] * 7,
        "expected_link8_base_position_m": [0.8, 0.0, 0.8],
        "expected_link8_base_orientation_xyzw": [0.5, -0.5, 0.5, -0.5],
        "maximum_joint_error_rad": 0.035,
        "maximum_position_error_m": 0.02,
        "maximum_orientation_error_deg": 5.0,
        "allow_task_space_equivalent_redundant_solution": True,
    }
    measured = {
        "joint_positions_rad": [0.5] + [0.0] * 6,
        "link8_base_position_m": [0.8, 0.0, 0.8],
        "link8_base_orientation_xyzw": [0.5, -0.5, 0.5, -0.5],
    }
    validate_errors(pose_errors(measured, reference), reference)
