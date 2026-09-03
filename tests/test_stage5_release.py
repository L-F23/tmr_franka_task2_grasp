import numpy as np

from stage5_release_diagonal import clearance_compensated_position, OrderedRelease, release_target


def test_release_target_moves_backward_and_down_together():
    target = release_target([1.0, 0.1, 0.8], 0.11, 0.01)
    assert np.allclose(target, [0.89, 0.1, 0.79])


def test_gripper_begins_at_6cm_and_arm_keeps_retracting():
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

    plan = [
        {"joint_positions_rad": [0.1] * 7, "position_m": [0.95, 0.0, 0.8]},
        {"joint_positions_rad": [0.2] * 7, "position_m": [0.94, 0.0, 0.8]},
        {"joint_positions_rad": [0.3] * 7, "position_m": [0.89, 0.0, 0.8]},
    ]
    progress = []
    gripper = OrderedRelease.retract_tilt_and_open(
        FakeExecutor(), plan, 1.0, 0.06, 0.05, 0.0,
        lambda kind, value: progress.append((kind, value)),
    )

    assert [event[0] for event in events] == [
        "arm", "arm", "gate", "gripper_start", "arm", "gate", "gripper_finish"
    ]
    assert gripper["reached_goal"] is True
    assert events[3] == ("gripper_start", 0.0)
    assert [kind for kind, _ in progress] == [
        "motion", "motion", "gripper_started", "motion", "gripper_finished"
    ]


def test_clearance_compensation_prevents_contact_end_drop():
    position, compensation = clearance_compensated_position(
        [0.9, 0.0, 0.80], [0.0, 0.70710678, 0.0, 0.70710678],
        [0.0, 0.0, 0.155], 0.79,
    )
    contact_z = (position + np.array([0.155, 0.0, 0.0]))[2]
    assert contact_z >= 0.79 - 1e-9
    assert compensation == 0.0
