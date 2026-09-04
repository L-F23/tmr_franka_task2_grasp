import pytest

from calibrate_wrist_lateral_mapping import fit_mapping


def test_fit_mapping_learns_right_correction_from_wrist_y_error():
    samples = [
        {"cumulative_actual_right_m": -0.03, "median_center_y_px": 210.0},
        {"cumulative_actual_right_m": -0.015, "median_center_y_px": 225.0},
        {"cumulative_actual_right_m": 0.0, "median_center_y_px": 240.0},
        {"cumulative_actual_right_m": 0.015, "median_center_y_px": 255.0},
        {"cumulative_actual_right_m": 0.03, "median_center_y_px": 270.0},
    ]
    result = fit_mapping(samples, 240.0)
    assert result["right_correction_m"]["linear_m_per_px"] == pytest.approx(-0.001)
    assert result["right_correction_m"]["quadratic_m_per_px2"] == pytest.approx(0.0)
    assert result["right_correction_m"]["rmse_m"] < 1e-9
