"""``controller_manager`` RT parameter file generation (issue #343).

The file produced here is what actually pins the UR driver's 500 Hz loop, so the
tests below lock the two ways it can fail *silently*:

* a malformed or mistyped file — ROS would reject or coerce it and the node
  would fall back to unpinned defaults, which is the pre-#343 behaviour the
  runtime verifier used to report as green. Every test that writes a file reads
  it back with ``yaml.safe_load`` and checks types, not just text.
* a guessed ``cpu_affinity`` — an unresolvable slot must drop the key, never
  emit the raw slot index (issue #163 pinned RT threads onto each other exactly
  that way).
"""

from __future__ import annotations

import pytest
import yaml

from rtc_tools.launch import cm_rt_params


def _load(path):
    with open(path) as handle:
        return yaml.safe_load(handle)["controller_manager"]["ros__parameters"]


# ── control_rate -> update_rate ──────────────────────────────────────────────


@pytest.mark.parametrize(
    ("control_rate", "expected"),
    [(500.0, 500), (500, 500), (250.0, 250), ("500.0", 500), (1000.0, 1000)],
)
def test_update_rate_accepts_whole_hz(control_rate, expected):
    assert cm_rt_params.update_rate_from_control_rate(control_rate) == expected


@pytest.mark.parametrize("control_rate", [499.5, 0.0, -500.0, 0.5])
def test_update_rate_rejects_unrepresentable(control_rate):
    """Fractional / non-positive rates must fail loudly, not truncate.

    ``update_rate`` is an int parameter: 499.5 would land as 499 and the CM loop
    would run at a rate the RTC controller does not share, forever.
    """
    with pytest.raises(ValueError):
        cm_rt_params.update_rate_from_control_rate(control_rate)


# ── control_rate lookup ──────────────────────────────────────────────────────


def test_control_rate_read_from_wildcard_block(tmp_path):
    cfg = tmp_path / "_base.yaml"
    cfg.write_text("/**:\n  ros__parameters:\n    control_rate: 500.0\n    other: 1\n")
    assert cm_rt_params.control_rate_from_yaml(str(cfg)) == 500.0


def test_control_rate_missing_raises(tmp_path):
    cfg = tmp_path / "_base.yaml"
    cfg.write_text("/**:\n  ros__parameters:\n    other: 1\n")
    with pytest.raises(KeyError):
        cm_rt_params.control_rate_from_yaml(str(cfg))


# ── cpu_affinity presence ────────────────────────────────────────────────────


def test_no_slot_omits_cpu_affinity(tmp_path):
    """``use_cpu_affinity:=false`` must not inject an affinity override."""
    path, params = cm_rt_params.write_cm_rt_param_file(
        str(tmp_path), control_rate=500.0, slot=None
    )
    assert "cpu_affinity" not in params
    assert "cpu_affinity" not in _load(path)


def test_slot_is_resolved_to_logical_cpu(tmp_path, monkeypatch):
    """The written value is the *logical cpu*, not the slot index it came from."""
    monkeypatch.setattr(
        cm_rt_params, "resolve_logical_cpu", lambda slot: 10 if slot == 6 else None
    )
    path, params = cm_rt_params.write_cm_rt_param_file(str(tmp_path), control_rate=500.0, slot=6)
    assert params["cpu_affinity"] == 10
    assert _load(path)["cpu_affinity"] == 10


def test_unresolvable_slot_omits_rather_than_guesses(tmp_path, monkeypatch):
    """A failed lookup drops the pin; emitting the raw slot is issue #163."""
    monkeypatch.setattr(cm_rt_params, "resolve_logical_cpu", lambda slot: None)
    path, params = cm_rt_params.write_cm_rt_param_file(str(tmp_path), control_rate=500.0, slot=6)
    assert "cpu_affinity" not in params
    written = _load(path)
    assert "cpu_affinity" not in written
    # The rest of the RT contract is unaffected — only affinity drops out.
    assert written["thread_priority"] == cm_rt_params.DEFAULT_THREAD_PRIORITY
    assert written["update_rate"] == 500


# ── file shape ───────────────────────────────────────────────────────────────


def test_written_file_is_valid_yaml_with_ros_types(tmp_path, monkeypatch):
    """Types must survive: ``lock_memory`` a bool, the rest ints.

    A quoted "true" or a float update_rate is a parameter-type mismatch at node
    start, which degrades to upstream defaults instead of failing visibly.
    """
    monkeypatch.setattr(cm_rt_params, "resolve_logical_cpu", lambda slot: 4)
    path, _ = cm_rt_params.write_cm_rt_param_file(
        str(tmp_path), control_rate=500.0, slot=2, thread_priority=50, lock_memory=True
    )
    written = _load(path)
    assert written == {
        "update_rate": 500,
        "cpu_affinity": 4,
        "thread_priority": 50,
        "lock_memory": True,
    }
    assert isinstance(written["lock_memory"], bool)
    assert isinstance(written["update_rate"], int)
    assert isinstance(written["cpu_affinity"], int)


def test_node_name_matches_upstream_parameter_scope(tmp_path):
    """The block must be ``controller_manager:`` — the node's own name.

    ``ros2_control_node`` reads this file as one of its parameter files; a
    different top-level key silently applies to nothing.
    """
    path, _ = cm_rt_params.write_cm_rt_param_file(str(tmp_path), control_rate=500.0)
    with open(path) as handle:
        loaded = yaml.safe_load(handle)
    assert list(loaded) == ["controller_manager"]
    assert "ros__parameters" in loaded["controller_manager"]


def test_lock_memory_can_be_disabled(tmp_path):
    path, _ = cm_rt_params.write_cm_rt_param_file(
        str(tmp_path), control_rate=500.0, lock_memory=False
    )
    assert _load(path)["lock_memory"] is False
