import json

import pytest

import quick_start
from run_full_thermal_pad_cycle import validate_prepared_record


def test_prepared_record_accepts_fresh_no_motion_record(tmp_path):
    path = tmp_path / "ready.json"
    path.write_text(json.dumps({
        "schema_version": 1,
        "status": "ready",
        "prepared_at_unix_s": 100.0,
        "physical_motion_commanded": False,
    }))
    assert validate_prepared_record(path, now=110.0)["status"] == "ready"


def test_prepared_record_rejects_stale_record(tmp_path):
    path = tmp_path / "ready.json"
    path.write_text(json.dumps({
        "schema_version": 1,
        "status": "ready",
        "prepared_at_unix_s": 100.0,
        "physical_motion_commanded": False,
    }))
    with pytest.raises(RuntimeError, match="stale"):
        validate_prepared_record(path, now=121.0)


def test_camera_gate_requires_advancing_main_and_left(monkeypatch):
    values = iter([
        {"healthy": {"main": True, "left": True},
         "sequence": {"main": 5, "left": 9},
         "frame_age_s": {"main": 0.1, "left": 0.1}},
        {"healthy": {"main": True, "left": True},
         "sequence": {"main": 5, "left": 10},
         "frame_age_s": {"main": 0.1, "left": 0.1}},
    ])
    monkeypatch.setattr(quick_start, "fetch_viewer_status", lambda: next(values))
    monkeypatch.setattr(quick_start.time, "sleep", lambda _seconds: None)
    with pytest.raises(quick_start.StartupBlocked, match="main"):
        quick_start.advancing_cameras()


def test_prepare_runs_independent_runtime_checks_and_sets_no_motion(monkeypatch, tmp_path):
    monkeypatch.setattr(quick_start, "ROS_ENV", tmp_path / "tmr_env.sh")
    quick_start.ROS_ENV.write_text("true\n")
    monkeypatch.setattr(quick_start, "READY_RECORD", tmp_path / "ready.json")
    monkeypatch.setattr(quick_start, "require_core_graph", lambda: {"label": "graph"})
    monkeypatch.setattr(quick_start, "ensure_base_runtime", lambda: {"label": "base"})
    monkeypatch.setattr(quick_start, "ensure_left_runtime", lambda: {"label": "left"})
    monkeypatch.setattr(quick_start, "ensure_viewer", lambda: {"label": "camera"})
    record = quick_start.prepare()
    assert record["status"] == "ready"
    assert record["physical_motion_commanded"] is False
    assert {item["label"] for item in record["results"]} == {
        "graph", "base", "left", "camera"
    }
