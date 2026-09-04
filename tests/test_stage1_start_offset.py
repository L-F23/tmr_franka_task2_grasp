import numpy as np

from set_stage1_start_from_current import contact_retreat_target, offset_targets


def test_offset_targets_use_ground_aligned_base_axes():
    backward, lowered = offset_targets([0.8, -0.1, 0.9], 0.06, 0.055)
    assert np.allclose(backward, [0.74, -0.1, 0.9])
    assert np.allclose(lowered, [0.74, -0.1, 0.845])


def test_offset_targets_allow_a_single_axis_move():
    backward, lowered = offset_targets([0.8, -0.1, 0.9], 0.03, 0.0)
    assert np.allclose(backward, [0.77, -0.1, 0.9])
    assert np.allclose(lowered, backward)


def test_offset_targets_allow_forward_motion():
    forward, lowered = offset_targets([0.8, -0.1, 0.9], 0.0, 0.0, 0.025)
    assert np.allclose(forward, [0.825, -0.1, 0.9])
    assert np.allclose(lowered, forward)


def test_offset_targets_allow_vertical_lift():
    after_x, lifted = offset_targets([0.8, -0.1, 0.9], 0.0, 0.0, 0.0, 0.12)
    assert np.allclose(after_x, [0.8, -0.1, 0.9])
    assert np.allclose(lifted, [0.8, -0.1, 1.02])


def test_contact_retreat_moves_back_along_ground_forward_axis():
    assert np.allclose(
        contact_retreat_target([0.88, -0.05, 0.84], 0.003),
        [0.877, -0.05, 0.84],
    )
