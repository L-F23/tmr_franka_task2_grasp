from stage1_close_gripper import reference_joints


def test_reference_joints_prefers_fresh_measured_after_pose():
    record = {
        "joint_positions_rad": [0.0] * 7,
        "after": {"joint_positions_rad": [0.2] * 7},
    }
    assert reference_joints(record) == [0.2] * 7


def test_reference_joints_supports_legacy_direct_record():
    assert reference_joints({"joint_positions_rad": [0.1] * 7}) == [0.1] * 7
