import numpy as np

from stage5_release_diagonal import OrderedRelease, release_target


def test_release_target_moves_backward_and_down_together():
    target = release_target([1.0, 0.1, 0.8], 0.11, 0.01)
    assert np.allclose(target, [0.89, 0.1, 0.79])


def test_gripper_opens_only_after_all_arm_waypoints_complete():
    events = []

    class FakeExecutor:
        def move_ptp(self, joints, label, speed):
            events.append(("arm", list(joints), label, speed))
            return {"label": label}

        def motion_gate(self):
            events.append(("gate",))

        def command_gripper(self, position, label):
            events.append(("gripper", position, label))
            return {"reached_goal": True}

    plan = [
        {"joint_positions_rad": [0.1] * 7},
        {"joint_positions_rad": [0.2] * 7},
    ]
    motions, gripper = OrderedRelease.retract_then_open(
        FakeExecutor(), plan, 0.012, 0.0
    )

    assert [event[0] for event in events] == ["arm", "arm", "gate", "gripper"]
    assert len(motions) == 2
    assert gripper["reached_goal"] is True
    assert events[-1][2] == "open_after_arm_motion_complete"
