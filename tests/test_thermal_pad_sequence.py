import math

import numpy as np
import pytest

from thermal_pad_sequence import (
    SequenceDesignError,
    build_sequence,
    transformed_axis,
)


CONFIG = {
    "ground_aligned_frame": "base",
    "shoulder_frame": "left_fr3v2_link0",
    "ground_forward_axis_xyz": [1.0, 0.0, 0.0],
    "ground_up_axis_xyz": [0.0, 0.0, 1.0],
    "link8_gripper_approach_axis_local_xyz": [0.0, 0.0, 1.0],
    "link8_gripper_opening_axis_local_xyz": [0.0, -1.0, 0.0],
    "segment_boundary_after_step": 6,
    "segment_1_terminal_target": "carry_far_12cm",
    "open_advance_m": 0.06,
    "staging_clearance_z_m": 0.08,
    "lift_vertical_m": 0.12,
    "carry_far_m": 0.12,
    "pre_place_lower_m": 0.22,
    "diagonal_down_m": 0.08,
    "diagonal_inward_m": 0.06,
    "release_inward_m": 0.05,
    "dump_toward_ground_deg": 15.0,
    "parameters_calibrated": False,
}


def targets_by_name(sequence):
    return {target["name"]: target for target in sequence["targets"]}


def test_sequence_preserves_requested_exact_displacements() -> None:
    grasp = np.array([1.0, 0.1, 0.75])
    sequence = build_sequence(grasp, CONFIG)
    target = targets_by_name(sequence)

    assert np.allclose(target["pick_height_retracted"]["position_m"], [0.94, 0.1, 0.75])
    assert np.allclose(target["advance_open_to_pad"]["position_m"], grasp)
    assert np.allclose(target["lift_vertical_12cm"]["position_m"], grasp + [0, 0, 0.12])
    assert np.allclose(
        np.asarray(target["carry_far_12cm"]["position_m"])
        - np.asarray(target["lift_vertical_12cm"]["position_m"]),
        [0.12, 0.0, 0.0],
    )
    assert np.allclose(
        np.asarray(target["lower_vertical_22cm"]["position_m"])
        - np.asarray(target["carry_far_12cm"]["position_m"]),
        [0.0, 0.0, -0.22],
    )


def test_pick_and_transfer_keep_horizontal_gripper_with_vertical_opening_axis() -> None:
    sequence = build_sequence([1.0, 0.0, 0.75], CONFIG)
    target = targets_by_name(sequence)
    for item in sequence["targets"][:-1]:
        approach = transformed_axis(
            item["orientation_xyzw"], CONFIG["link8_gripper_approach_axis_local_xyz"]
        )
        opening = transformed_axis(
            item["orientation_xyzw"], CONFIG["link8_gripper_opening_axis_local_xyz"]
        )
        assert np.allclose(approach, [1.0, 0.0, 0.0])
        assert np.allclose(opening, [0.0, 0.0, 1.0])


def test_release_opens_then_dumps_toward_ground_while_retracting() -> None:
    sequence = build_sequence([1.0, 0.0, 0.75], CONFIG)
    target = targets_by_name(sequence)
    diagonal = np.asarray(target["lower_and_retract"]["position_m"])
    release_target = target["dump_toward_ground_and_retract"]
    release = np.asarray(release_target["position_m"])
    assert np.allclose(release - diagonal, [-0.05, 0.0, 0.0])

    approach = transformed_axis(
        release_target["orientation_xyzw"], CONFIG["link8_gripper_approach_axis_local_xyz"]
    )
    assert approach[0] == pytest.approx(math.cos(math.radians(15.0)), abs=1e-7)
    assert approach[2] == pytest.approx(-math.sin(math.radians(15.0)), abs=1e-7)
    events = sequence["events"]
    assert events[-2]["command"] == "open_gripper"
    assert events[-1]["after_target"] == "dump_toward_ground_and_retract"


def test_unvalidated_release_parameters_block_execution() -> None:
    sequence = build_sequence([1.0, 0.0, 0.75], CONFIG)
    assert not sequence["execution_ready"]
    assert sequence["execution_blockers"]
    assert [event["command"] for event in sequence["events"]] == [
        "open_gripper", "close_gripper", "verify_object_held",
        "hold_and_end_segment_1", "authorize_segment_2",
        "open_gripper", "verify_release",
    ]


def test_sequence_is_split_after_step_six() -> None:
    sequence = build_sequence([1.0, 0.0, 0.75], CONFIG)
    first, second = sequence["segments"]
    assert first["steps"] == [1, 2, 3, 4, 5, 6]
    assert first["terminal_target"] == "carry_far_12cm"
    assert first["terminal_behavior"].startswith("hold_pose")
    assert second["steps"] == [7, 8, 9, 10]
    assert second["target_names"][0] == "lower_vertical_22cm"
    assert all(target["segment"] == 1 for target in sequence["targets"][:5])
    assert all(target["segment"] == 2 for target in sequence["targets"][5:])


def test_non_orthogonal_gripper_axes_are_rejected() -> None:
    invalid = dict(CONFIG, link8_gripper_opening_axis_local_xyz=[0.0, 0.0, 1.0])
    with pytest.raises(SequenceDesignError, match="must be orthogonal"):
        build_sequence([1.0, 0.0, 0.75], invalid)


def test_shoulder_frame_cannot_be_used_as_ground_reference() -> None:
    invalid = dict(CONFIG, ground_aligned_frame="left_fr3v2_link0")
    with pytest.raises(SequenceDesignError, match="distinct from the arm shoulder"):
        build_sequence([1.0, 0.0, 0.75], invalid)
