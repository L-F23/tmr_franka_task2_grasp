from run_full_thermal_pad_cycle import FULL_STAGE_ORDER


def test_full_cycle_places_transport_before_black_base_alignment():
    assert FULL_STAGE_ORDER.index("base_right_2m_complete") < FULL_STAGE_ORDER.index(
        "black_base_and_thermal_pad_centered"
    )


def test_isolated_base_runtime_starts_before_transport():
    assert FULL_STAGE_ORDER.index("isolated_base_runtime_ready") < FULL_STAGE_ORDER.index(
        "base_right_2m_complete"
    )


def test_grasp_pose_requires_lateral_confirmation_before_closing():
    pose = FULL_STAGE_ORDER.index("left_grasp_pose_reached")
    aligned = FULL_STAGE_ORDER.index("pregrasp_lateral_alignment_confirmed")
    closed = FULL_STAGE_ORDER.index("thermal_pad_grasped")
    assert aligned + 1 == pose
    assert pose + 1 == closed


def test_release_order_has_clearance_before_restore():
    release = FULL_STAGE_ORDER.index(
        "placement_retract_tilt_with_gripper_held"
    )
    clearance = FULL_STAGE_ORDER.index("gripper_open_then_vertical_clearance_5cm")
    restore = FULL_STAGE_ORDER.index("left_initial_restored")
    assert release + 1 == clearance
    assert clearance + 1 == restore
