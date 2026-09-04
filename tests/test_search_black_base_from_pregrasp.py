import numpy as np
import cv2
import pytest

from search_black_base_from_pregrasp import command_right_step, observe_reference


def test_fresh_reference_is_recognized_at_zero_error():
    image = np.full((480, 640, 3), 180, np.uint8)
    cv2.rectangle(image, (290, 160), (539, 314), (20, 20, 20), -1)
    cv2.circle(image, (410, 230), 28, (115, 115, 115), -1)
    result = observe_reference(image, image, (290, 160, 250, 155), 0.72, 20)
    assert result["near_reference"] is True
    assert result["center_error_px"] == pytest.approx([0.0, 0.0])


def test_right_search_refuses_to_continue_after_overshoot():
    with pytest.raises(RuntimeError, match="passed the reference"):
        command_right_step(
            {"visible": True, "center_error_px": [0.0, 25.0]}, 0.08
        )


def test_visible_target_switches_to_fine_step():
    assert command_right_step(
        {"visible": True, "center_error_px": [0.0, -60.0]}, 0.08
    ) == pytest.approx(0.02)
