from run_task2_from_initial import STAGE_ORDER, stage_commands


def command_for(label: str) -> list[str]:
    return next(command for stage, command, _timeout in stage_commands() if stage == label)


def test_task2_initial_transition_is_reset_then_pregrasp_then_verify():
    health = STAGE_ORDER.index("services_and_live_cameras_verified")
    spine = STAGE_ORDER.index("spine_restored_0_6m")
    right = STAGE_ORDER.index("right_arm_parking_restored")
    reset = STAGE_ORDER.index("left_initial_restored")
    pregrasp = STAGE_ORDER.index("left_pregrasp_reached")
    verified = STAGE_ORDER.index("calibrated_pregrasp_pose_verified")
    aligned = STAGE_ORDER.index("pregrasp_lateral_alignment_confirmed")
    assert health + 1 == spine
    assert spine + 1 == right
    assert right + 1 == reset
    assert reset + 1 == pregrasp
    assert pregrasp + 1 == verified
    assert verified + 1 == aligned


def test_spine_reset_uses_dedicated_spine_only_stage():
    command = command_for("spine_restored_0_6m")
    assert command[2].endswith("reset_spine_to_task_height.py")
    assert "--execute" in command


def test_right_arm_is_restored_to_recorded_parking_pose_before_left_reset():
    command = command_for("right_arm_parking_restored")
    assert command[2].endswith("restore_right_parking_direct.py")
    assert STAGE_ORDER.index("right_arm_parking_restored") < STAGE_ORDER.index(
        "left_initial_restored"
    )


def test_pregrasp_transition_is_stage_start_only_and_does_not_grip():
    command = command_for("left_pregrasp_reached")
    assert command[2].endswith("execute_thermal_pad_grasp.py")
    assert "--stage-start-only" in command
    assert "--empty-cycle" in command


def test_initial_runner_enforces_pregrasp_lateral_alignment():
    alignment = command_for("pregrasp_lateral_alignment_confirmed")
    assert alignment[2].endswith("black_base_pose_alignment.py")
    assert "--execute" in alignment
    close = command_for("thermal_pad_grasped")
    assert "--skip-pregrasp-calibration-gates" not in close


def test_initial_runner_omits_transport_and_coarse_search():
    arguments = [
        argument
        for _label, command, _timeout in stage_commands()
        for argument in command
    ]
    assert "run_full_thermal_pad_cycle.py" not in arguments
    assert "align_to_thermal_pad.py" not in arguments
    assert "table_edge_positioning.py" not in arguments
    assert not any(
        argument in arguments
        for argument in ("--base-right-m", "--right-m", "--left-m")
    )


def test_final_stage_restores_left_arm_after_transfer():
    assert STAGE_ORDER[-1] == "left_initial_restored_after_transfer"
    assert command_for(STAGE_ORDER[-1])[2].endswith("restore_left_initial_direct.py")
