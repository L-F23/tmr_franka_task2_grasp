from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "policy" / "base_runtime" / "cmd_vel_adapter.py").read_text(
    encoding="utf-8"
)


def test_adapter_is_fail_closed_and_zero_latching():
    assert 'self._lease_active = False' in SOURCE
    assert 'if not self._lease_active or not self._finite(message.twist)' in SOURCE
    assert 'time.monotonic() - self._last_command_at > 0.50' in SOURCE
    assert 'self._publish_zero()' in SOURCE


def test_adapter_rejects_replayed_or_unframed_commands():
    assert 'message.header.frame_id.lstrip("/") != "base_link"' in SOURCE
    assert 'age_s > 0.35' in SOURCE
    assert 'age_s < -0.10' in SOURCE


def test_adapter_owns_only_the_task2_mission_input():
    assert '"/tmr_cycle/mission_cmd_vel"' in SOURCE
    assert '"/tmr_cycle/mission_active"' in SOURCE
    assert '"/swerve_drive_controller/cmd_vel"' in SOURCE
    assert '"/cmd_vel"' not in SOURCE


def test_motion_workers_release_the_lease_after_zeroing():
    for relative in (
        "policy/guarded_lateral_step.py",
        "policy/stage0_wall_docking_base.py",
    ):
        source = (ROOT / relative).read_text(encoding="utf-8")
        assert "lease.data = False" in source
        assert source.index("self.publish(0.0, 0.0, 0.0)") < source.index(
            "lease.data = False"
        )


def test_runtime_requires_the_drive_controller():
    runtime = (ROOT / "policy/base_runtime/ensure_runtime.sh").read_text(
        encoding="utf-8"
    )
    assert "/swerve_drive_controller/cmd_vel" in runtime
    assert "controller_has_subscriber" in runtime
