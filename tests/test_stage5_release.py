import numpy as np

from stage5_release_diagonal import (
    OrderedRelease,
    clearance_compensated_position,
    front_loaded_descent_fraction,
    release_target,
)


def test_release_target_moves_backward_and_down_together():
    target = release_target([1.0, 0.1, 0.8], 0.11, 0.01)
    assert np.allclose(target, [0.89, 0.1, 0.79])


def test_initial_and_rotating_descent_total_70mm():
    start = np.array([1.0, 0.1, 0.8])
    lowered = release_target(start, 0.0, 0.008)
    target = release_target(lowered, 0.11, 0.062)
    assert np.allclose(lowered, [1.0, 0.1, 0.792])
    assert np.allclose(target, [0.89, 0.1, 0.73])


def test_descent_is_front_loaded_but_preserves_total_distance():
    fractions = [front_loaded_descent_fraction(value / 10.0) for value in range(11)]
    increments = np.diff(fractions)
    assert fractions[0] == 0.0
    assert fractions[-1] == 1.0
    assert np.all(increments > 0.0)
    assert increments[0] > increments[-1]
    assert np.all(np.diff(increments) < 1e-12)


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


def test_slow_gripper_open_is_split_across_remaining_retreat():
    events = []

    class FakeExecutor:
        def move_ptp(self, joints, label, speed):
            events.append(("arm", label))
            return {"label": label}

        def motion_gate(self):
            events.append(("gate",))

        def command_gripper(self, position, label):
            events.append(("gripper", position, label))
            return {"position": position, "reached_goal": True}

    plan = [
        {"joint_positions_rad": [value] * 7, "position_m": [x, 0.0, 0.8]}
        for value, x in zip((0.1, 0.2, 0.3, 0.4), (0.94, 0.92, 0.90, 0.88))
    ]
    progress = []
    result = OrderedRelease.retract_tilt_and_open(
        FakeExecutor(), plan, 1.0, 0.06, 0.05, 0.4,
        lambda kind, value: progress.append((kind, value)),
        opening_start_position=0.8, gripper_open_steps=4,
    )

    gripper_targets = [event[1] for event in events if event[0] == "gripper"]
    assert np.allclose(gripper_targets, [0.7, 0.6, 0.5, 0.4])
    assert result["position"] == 0.4
    assert [kind for kind, _ in progress].count("gripper_step") == 4


def test_clearance_compensation_prevents_contact_end_drop():
    position, compensation = clearance_compensated_position(
        [0.9, 0.0, 0.80], [0.0, 0.70710678, 0.0, 0.70710678],
        [0.0, 0.0, 0.155], 0.79,
    )
    contact_z = (position + np.array([0.155, 0.0, 0.0]))[2]
    assert contact_z >= 0.79 - 1e-9
    assert compensation == 0.0


def test_open_result_accepts_measured_zero_even_if_action_flag_is_false(monkeypatch):
    monkeypatch.setattr(
        "stage5_release_diagonal.rclpy.spin_until_future_complete",
        lambda *args, **kwargs: None,
    )
    class Result:
        position = 0.0
        effort = 1.0
        stalled = False
        reached_goal = False

    class Wrapped:
        status = 6
        result = Result()

    class Future:
        def done(self):
            return True

        def result(self):
            return Wrapped()

    class Handle:
        def get_result_async(self):
            return Future()

    class Fake:
        requested_open_position = 0.0

    report = OrderedRelease.finish_opening(Fake(), Handle())
    assert report["measured_open_position_accepted"] is True
