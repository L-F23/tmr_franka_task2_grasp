import math

import numpy as np
import pytest

from thermal_pad_sequence import (
    SequenceDesignError,
    build_sequence,
    tool_down_axis,
)


CONFIG = {
    "ground_aligned_frame": "base",
    "shoulder_frame": "left_fr3v2_link0",
    "ground_forward_axis_xyz": [1.0, 0.0, 0.0],
    "segment_boundary_after_step": 6,
    "segment_1_terminal_target": "carry_far_12cm",
    "open_advance_m": 0.06,
    "staging_clearance_z_m": 0.08,
    "lift_vertical_m": 0.22,
    "carry_far_m": 0.12,
    "pre_place_lower_m": 0.02,
    "diagonal_down_m": 0.08,
    "diagonal_inward_m": 0.06,
    "release_inward_m": 0.05,
    "release_forward_tilt_deg": 25.0,
    "maximum_tool_vertical_error_deg": 5.0,
    "parameters_calibrated": False,
}


def targets_by_name(sequence):
    return {target["name"]: target for target in sequence["targets"]}


def test_sequence_preserves_requested_exact_displacements() -> None:
    grasp = np.array([1.0, 0.1, 0.75])
    # Local tool +Z points down; its flange plane is horizontal to the table.
    vertical = [1.0, 0.0, 0.0, 0.0]
    sequence = build_sequence(grasp, vertical, CONFIG)
    target = targets_by_name(sequence)

    assert np.allclose(target["pick_height_retracted"]["position_m"], [0.94, 0.1, 0.75])
    assert np.allclose(target["advance_open_to_pad"]["position_m"], grasp)
    assert np.allclose(target["lift_vertical_22cm"]["position_m"], grasp + [0, 0, 0.22])
    assert np.allclose(
        np.asarray(target["carry_far_12cm"]["position_m"])
        - np.asarray(target["lift_vertical_22cm"]["position_m"]),
        [0.12, 0.0, 0.0],
    )
    assert np.allclose(
        np.asarray(target["lower_vertical_2cm"]["position_m"])
        - np.asarray(target["carry_far_12cm"]["position_m"]),
        [0.0, 0.0, -0.02],
    )


def test_release_combines_forward_tilt_and_inward_translation() -> None:
    sequence = build_sequence([1.0, 0.0, 0.75], [1.0, 0.0, 0.0, 0.0], CONFIG)
    target = targets_by_name(sequence)
    diagonal = np.asarray(target["lower_and_retract"]["position_m"])
    release = np.asarray(target["tilt_forward_and_retract_release"]["position_m"])
    assert np.allclose(release - diagonal, [-0.05, 0.0, 0.0])

    release_down = tool_down_axis(target["tilt_forward_and_retract_release"]["orientation_xyzw"])
    assert release_down[0] == pytest.approx(math.sin(math.radians(25.0)), abs=1e-7)
    assert release_down[2] == pytest.approx(-math.cos(math.radians(25.0)), abs=1e-7)


def test_unvalidated_release_parameters_block_execution() -> None:
    sequence = build_sequence([1.0, 0.0, 0.75], [1.0, 0.0, 0.0, 0.0], CONFIG)
    assert not sequence["execution_ready"]
    assert sequence["execution_blockers"]
    assert [event["command"] for event in sequence["events"]] == [
        "open_gripper", "close_gripper", "verify_object_held",
        "hold_and_end_segment_1", "authorize_segment_2",
        "open_gripper", "verify_release",
    ]


def test_sequence_is_split_after_step_six() -> None:
    sequence = build_sequence([1.0, 0.0, 0.75], [1.0, 0.0, 0.0, 0.0], CONFIG)
    first, second = sequence["segments"]
    assert first["steps"] == [1, 2, 3, 4, 5, 6]
    assert first["terminal_target"] == "carry_far_12cm"
    assert first["terminal_behavior"].startswith("hold_pose")
    assert second["steps"] == [7, 8, 9, 10]
    assert second["target_names"][0] == "lower_vertical_2cm"
    assert all(target["segment"] == 1 for target in sequence["targets"][:5])
    assert all(target["segment"] == 2 for target in sequence["targets"][5:])


def test_non_vertical_gripper_is_rejected() -> None:
    with pytest.raises(SequenceDesignError, match="not vertical"):
        build_sequence([1.0, 0.0, 0.75], [0.0, 0.0, 0.0, 1.0], CONFIG)


def test_shoulder_frame_cannot_be_used_as_ground_reference() -> None:
    invalid = dict(CONFIG, ground_aligned_frame="left_fr3v2_link0")
    with pytest.raises(SequenceDesignError, match="distinct from the arm shoulder"):
        build_sequence([1.0, 0.0, 0.75], [1.0, 0.0, 0.0, 0.0], invalid)
