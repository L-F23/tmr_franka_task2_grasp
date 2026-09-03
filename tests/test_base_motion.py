import pytest

from base_motion import guarded_transport, split_lateral_move


def test_two_meter_transport_is_split_into_guarded_steps():
    steps = split_lateral_move(2.0)
    assert len(steps) == 25
    assert sum(steps) == pytest.approx(2.0)
    assert all(0.008 <= abs(step) <= 0.08 for step in steps)


def test_split_preserves_negative_direction():
    steps = split_lateral_move(-0.195)
    assert sum(steps) == pytest.approx(-0.195)
    assert all(step < 0.0 for step in steps)


def test_transport_compensates_accumulated_short_step_error():
    commands = []

    def short_mover(command_m):
        commands.append(command_m)
        actual_m = command_m - 0.0045 if command_m > 0.0 else command_m + 0.0045
        return {"status": "success", "actual_right_m": actual_m}

    results, actual_m = guarded_transport(2.0, mover=short_mover)

    assert abs(actual_m - 2.0) <= 0.005
    assert len(results) > 25
    assert all(0.008 <= abs(command) <= 0.08 for command in commands)
