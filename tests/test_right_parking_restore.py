import json
from pathlib import Path

import pytest

from restore_right_parking_direct import parking_parameters


ROOT = Path(__file__).resolve().parents[1]


def test_recorded_right_parking_pose_is_enabled_and_complete():
    config = json.loads((ROOT / "config" / "initial_pose.json").read_text())
    target, speed, tolerance = parking_parameters(config)
    assert len(target) == 7
    assert target == pytest.approx([
        -1.3082223883915827,
        -1.221499396341589,
        0.4977228788663176,
        -2.8806021162096016,
        -0.4502475342569809,
        2.1810914572975415,
        1.1374866925775369,
    ])
    assert speed == pytest.approx(0.06)
    assert tolerance == pytest.approx(0.012)


def test_right_parking_restore_rejects_disabled_commands():
    with pytest.raises(ValueError, match="disabled"):
        parking_parameters({
            "right_arm": {
                "policy": "restore_recorded_parking_pose",
                "commands_allowed_during_initialization": False,
                "target_positions_rad": [0.0] * 7,
                "maximum_velocity_rad_s": 0.06,
                "maximum_final_error_rad": 0.012,
            }
        })
