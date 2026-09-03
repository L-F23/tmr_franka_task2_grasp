import numpy as np

from stage5_release_diagonal import release_target


def test_release_target_moves_backward_and_down_together():
    target = release_target([1.0, 0.1, 0.8], 0.11, 0.01)
    assert np.allclose(target, [0.89, 0.1, 0.79])
