"""tools/standing_eval/twist_script.py (not installed — read from the source tree).

The script's time axes: the wall clock scaled by --rtf (the default) and the
sim clock of an MPC lock-step run (the latest /sim/time minus its value at the
script's start). The ROS loop itself is not run here.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

_PATH = Path(__file__).resolve().parents[1] / "tools" / "standing_eval" / "twist_script.py"
_SPEC = importlib.util.spec_from_file_location("standing_eval_twist_script", _PATH)
twist_script = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(twist_script)


def test_the_sim_axis_is_zero_at_the_start_and_follows_the_sim_time():
    axis = twist_script.SimAxis()
    assert axis.elapsed() == 0.0  # nothing heard yet
    axis.observe(250.0)
    axis.start()
    assert axis.elapsed() == 0.0
    axis.observe(250.75)
    assert axis.elapsed() == pytest.approx(0.75)
    axis.observe(262.0)  # the sim ran a long step: the script's time jumps with it
    assert axis.elapsed() == pytest.approx(12.0)


def test_the_sim_axis_wakes_for_an_edge_within_its_poll_interval():
    axis = twist_script.SimAxis()
    axis.observe(10.0)
    axis.start()
    axis.observe(10.499)
    assert axis.wake_in(0.5) == pytest.approx(0.001)  # the edge is 1 ms of sim away
    assert axis.wake_in(5.0) == twist_script.SimAxis.POLL_S
    assert axis.wake_in(0.1) == 0.0  # already past


def test_the_wall_axis_scales_by_the_rtf(monkeypatch):
    now = {"t": 1000.0}
    monkeypatch.setattr(twist_script.time, "monotonic", lambda: now["t"])
    axis = twist_script.WallAxis(0.5)
    now["t"] += 4.0
    assert axis.elapsed() == pytest.approx(2.0)
    assert axis.wake_in(3.0) == pytest.approx(2.0)  # 1 more sim second = 2 wall seconds
