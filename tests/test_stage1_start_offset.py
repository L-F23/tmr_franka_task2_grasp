import numpy as np

from set_stage1_start_from_current import offset_targets


def test_offset_targets_use_ground_aligned_base_axes():
    backward, lowered = offset_targets([0.8, -0.1, 0.9], 0.06, 0.055)
    assert np.allclose(backward, [0.74, -0.1, 0.9])
    assert np.allclose(lowered, [0.74, -0.1, 0.845])


def test_offset_targets_allow_a_single_axis_move():
    backward, lowered = offset_targets([0.8, -0.1, 0.9], 0.03, 0.0)
    assert np.allclose(backward, [0.77, -0.1, 0.9])
    assert np.allclose(lowered, backward)
