from run_full_thermal_pad_cycle import FULL_STAGE_ORDER


def test_full_cycle_places_transport_before_black_base_alignment():
    assert FULL_STAGE_ORDER.index("base_right_2m_complete") < FULL_STAGE_ORDER.index(
        "black_base_and_thermal_pad_centered"
    )


def test_isolated_base_runtime_starts_before_transport():
    assert FULL_STAGE_ORDER.index("isolated_base_runtime_ready") < FULL_STAGE_ORDER.index(
        "base_right_2m_complete"
    )


def test_release_order_is_move_then_open_then_restore():
    release = FULL_STAGE_ORDER.index(
        "retract_11cm_down_1cm_complete_then_gripper_open"
    )
    restore = FULL_STAGE_ORDER.index("left_initial_restored")
    assert release + 1 == restore
