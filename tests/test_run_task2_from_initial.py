from run_task2_from_initial import STAGE_ORDER, stage_commands


def command_for(label: str) -> list[str]:
    return next(command for stage, command, _timeout in stage_commands() if stage == label)


def test_task2_initial_transition_is_reset_then_pregrasp_then_verify():
    health = STAGE_ORDER.index("services_and_live_cameras_verified")
    reset = STAGE_ORDER.index("left_initial_restored")
    pregrasp = STAGE_ORDER.index("left_pregrasp_reached")
    verified = STAGE_ORDER.index("calibrated_pregrasp_pose_verified")
    assert health + 1 == reset
    assert reset + 1 == pregrasp
    assert pregrasp + 1 == verified


def test_pregrasp_transition_is_stage_start_only_and_does_not_grip():
    command = command_for("left_pregrasp_reached")
    assert command[2].endswith("execute_thermal_pad_grasp.py")
    assert "--stage-start-only" in command
    assert "--empty-cycle" in command


def test_initial_runner_omits_transport_and_coarse_search():
    arguments = [
        argument
        for _label, command, _timeout in stage_commands()
        for argument in command
    ]
    assert "run_full_thermal_pad_cycle.py" not in arguments
    assert "align_to_thermal_pad.py" not in arguments
    assert "table_edge_positioning.py" not in arguments
    assert "2.0" not in arguments


def test_final_stage_restores_left_arm_after_transfer():
    assert STAGE_ORDER[-1] == "left_initial_restored_after_transfer"
    assert command_for(STAGE_ORDER[-1])[2].endswith("restore_left_initial_direct.py")
