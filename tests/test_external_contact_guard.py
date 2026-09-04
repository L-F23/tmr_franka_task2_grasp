from types import SimpleNamespace

from execute_thermal_pad_grasp import ExternalContactGuard


def vector(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def state(*, force=(0.0, 0.0, 0.0), torque=(0.0, 0.0, 0.0), joint=None,
          native_contact=False):
    indicators = SimpleNamespace(
        is_cartesian_linear_contact=vector(1.0 if native_contact else 0.0),
        is_cartesian_angular_contact=vector(),
        is_cartesian_linear_collision=vector(),
        is_cartesian_angular_collision=vector(),
        is_joint_contact=[0.0] * 7,
        is_joint_collision=[0.0] * 7,
    )
    return SimpleNamespace(
        o_f_ext_hat_k=SimpleNamespace(wrench=SimpleNamespace(
            force=vector(*force), torque=vector(*torque),
        )),
        tau_ext_hat_filtered=SimpleNamespace(effort=joint or [0.0] * 7),
        collision_indicators=indicators,
    )


def guard(**kwargs):
    return ExternalContactGuard([0.0] * 3, [0.0] * 3, [0.0] * 7, **kwargs)


def test_guard_ignores_subthreshold_bias_and_one_sample_spike():
    item = guard(consecutive_samples=3)
    assert item.observe(state(force=(3.9, 0.0, 0.0))) is None
    assert item.observe(state(force=(4.5, 0.0, 0.0))) is None
    assert item.observe(state(force=(0.0, 0.0, 0.0))) is None
    assert item.trigger_count == 0


def test_guard_triggers_after_three_force_samples():
    item = guard(consecutive_samples=3)
    assert item.observe(state(force=(4.5, 0.0, 0.0))) is None
    assert item.observe(state(force=(4.5, 0.0, 0.0))) is None
    result = item.observe(state(force=(4.5, 0.0, 0.0)))
    assert result is not None
    assert "approach_axis_force_delta" in result["candidate_reasons"]


def test_native_franka_contact_flag_is_debounced_then_triggers():
    item = guard(consecutive_samples=2)
    assert item.observe(state(native_contact=True)) is None
    result = item.observe(state(native_contact=True))
    assert result is not None
    assert result["native_contact_or_collision"] is True


def test_torque_only_transient_is_diagnostic_but_cannot_stop_forward_approach():
    item = guard(torque_delta_norm_nm=0.8, consecutive_samples=3)
    for _ in range(8):
        assert item.observe(state(torque=(0.0, 0.81, 0.0))) is None
    assert item.last_measurement["candidate_reasons"] == ["torque_delta_norm"]
    assert item.last_measurement["trigger_eligible"] is False
    assert item.trigger_count == 0
