from run_from_ccw_restore_to_finish import (
    BACKWARD_M,
    MAXIMUM_SEARCH_RIGHT_M,
    PRESEARCH_RIGHT_M,
    RESTORE_CCW_DEG,
    ROUTE_STAGES,
)


def test_route_parameters_match_operator_sequence():
    assert RESTORE_CCW_DEG == 90.0
    assert BACKWARD_M == 0.55
    assert PRESEARCH_RIGHT_M == 1.40
    assert MAXIMUM_SEARCH_RIGHT_M == 1.50


def test_route_orders_search_before_wall_and_wrist_alignment():
    assert ROUTE_STAGES.index("search_right_until_target_or_150cm") < ROUTE_STAGES.index(
        "rear_wall_baseline_restored"
    )
    assert ROUTE_STAGES.index("rear_wall_baseline_restored") < ROUTE_STAGES.index(
        "robust_black_base_pose_alignment_confirmed"
    )
    assert ROUTE_STAGES[-1] == "grasp_transport_place_and_left_restore"
