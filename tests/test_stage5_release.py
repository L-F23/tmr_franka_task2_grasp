import numpy as np

from stage5_release_diagonal import OrderedRelease, release_target


def test_release_target_moves_backward_and_down_together():
    target = release_target([1.0, 0.1, 0.8], 0.11, 0.01)
    assert np.allclose(target, [0.89, 0.1, 0.79])


def test_gripper_begins_at_halfway_and_finishes_after_arm():
    events = []

    class FakeExecutor:
        def move_ptp(self, joints, label, speed):
            events.append(("arm", list(joints), label, speed))
            return {"label": label}

        def motion_gate(self):
            events.append(("gate",))

        def begin_opening(self, position):
            events.append(("gripper_start", position))
            return "handle"

        def finish_opening(self, handle):
            events.append(("gripper_finish", handle))
            return {"reached_goal": True}

    first_half = [{"joint_positions_rad": [0.1] * 7}]
    second_half = [{"joint_positions_rad": [0.2] * 7}]
    motions, gripper = OrderedRelease.retract_tilt_and_open(
        FakeExecutor(), first_half, second_half, 0.012, 0.0
    )

    assert [event[0] for event in events] == [
        "arm", "gate", "gripper_start", "arm", "gate", "gripper_finish"
    ]
    assert len(motions) == 2
    assert gripper["reached_goal"] is True
    assert events[2] == ("gripper_start", 0.0)
