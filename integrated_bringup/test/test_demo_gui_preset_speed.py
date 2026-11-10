"""Unit tests for the hand speed of a demo GUI preset send.

A preset's ``grasp_time`` becomes the hand trajectory speed of the controller
the goal goes to. What these tests hold:

- the parameter names per controller, read against the C++ that declares them
  (the GUI set ``hand_trajectory_speed`` on ``hand_joint_controller``, which
  declares ``trajectory_speed``, and every preset send was refused),
- the speed plan: lowered to the read-only cap instead of sent and refused, and
  an honest lower bound on the duration (quintic peak = 15/8 of the mean),
- the ``app.py`` wiring pytest can see: the goal is published from the
  completion of the set, on the Tk thread, and exactly once.

Tk layout is checked at runtime, as for every other panel.
"""

from __future__ import annotations

import inspect
import math
import re
from pathlib import Path

import pytest

from integrated_bringup.demo_gui import config
from integrated_bringup.demo_gui.hand_joint import HAND_JOINT_OWNERS
from integrated_bringup.demo_gui.preset_speed import (
    DEMO_HAND_SPEED_PARAMS,
    FALLBACK_SPEED,
    HAND_JOINT_SPEED_PARAMS,
    QUINTIC_PEAK_FACTOR,
    describe_plan,
    hand_speed_params,
    plan_hand_speed,
)

_CONTROLLERS = Path(__file__).resolve().parents[1] / "src" / "controllers"

# Controller key → the directory whose parameters.cpp declares its parameters.
_PARAMETER_SOURCES = {
    "demo_joint_controller": "joint",
    "demo_task_controller": "task",
    "demo_compliance_controller": "compliance",
    "demo_wbc_controller": "wbc",
    "hand_joint_controller": "hand_joint",
}


def _params(ctrl: str):
    return hand_speed_params(ctrl, config.GAIN_PARAM_DISPATCH, HAND_JOINT_OWNERS)


# ── Parameter names ──────────────────────────────────────────────────────────


@pytest.mark.parametrize("ctrl", sorted(_PARAMETER_SOURCES))
def test_speed_params_are_the_ones_the_controller_declares(ctrl):
    names = _params(ctrl)
    assert names is not None
    source = (_CONTROLLERS / _PARAMETER_SOURCES[ctrl] / "parameters.cpp").read_text()
    declared = set(re.findall(r'declare(?:_double)?\(\s*"([^"]+)"', source))
    assert set(names) <= declared, (names, sorted(declared))


def test_hand_joint_controller_does_not_get_the_demo_spelling():
    assert _params("hand_joint_controller") == HAND_JOINT_SPEED_PARAMS
    assert HAND_JOINT_SPEED_PARAMS == ("trajectory_speed", "max_traj_velocity")
    assert _params("demo_joint_controller") == DEMO_HAND_SPEED_PARAMS
    # demo_task_controller declares BOTH trajectory_speed (the arm's) and
    # hand_trajectory_speed: the hand one is the preset's.
    assert _params("demo_task_controller") == DEMO_HAND_SPEED_PARAMS


def test_the_dualarm_rows_are_recognised_by_their_declared_name():
    """Its schema is registered at GUI start from the YAML; the pair is found
    in whatever rows that produced, and is what parameters.cpp declares."""
    from integrated_bringup.demo_gui.preset_speed import DUALARM_HAND_SPEED_PARAMS

    dispatch = {
        "demo_dualarm_controller": {
            "hand speed": (DUALARM_HAND_SPEED_PARAMS[0], config._set_double),
            "hand speed max": (DUALARM_HAND_SPEED_PARAMS[1], config._read_only),
        }
    }
    assert (
        hand_speed_params("demo_dualarm_controller", dispatch, HAND_JOINT_OWNERS)
        == DUALARM_HAND_SPEED_PARAMS
    )
    source = (_CONTROLLERS / "dualarm" / "parameters.cpp").read_text()
    for name in DUALARM_HAND_SPEED_PARAMS:
        assert f'"{name}"' in source, name


def test_a_controller_without_a_hand_speed_gets_none():
    assert _params("demo_inference_controller") is None
    assert _params("no_such_controller") is None


def test_every_static_gain_schema_with_a_hand_speed_also_shows_its_cap():
    for ctrl, dispatch in config.GAIN_PARAM_DISPATCH.items():
        names = _params(ctrl)
        if names is None:
            continue
        declared = dict(dispatch.values())
        assert declared[names[1]] is config._read_only, ctrl
        assert declared[names[0]] is config._set_double, ctrl


# ── Speed plan ───────────────────────────────────────────────────────────────


def test_within_the_cap_the_speed_is_distance_over_time():
    plan = plan_hand_speed(max_dist=1.0, grasp_time=2.0, cap=2.0)
    assert plan.speed == pytest.approx(0.5)
    assert not plan.capped
    # 1.875 * 1.0 / 2.0 = 0.9375 s < 2 s: grasp_time is met.
    assert plan.min_duration == pytest.approx(2.0)


def test_above_the_cap_the_speed_is_lowered_not_sent_to_be_refused():
    plan = plan_hand_speed(max_dist=1.0, grasp_time=0.2, cap=2.0)
    assert plan.speed == pytest.approx(2.0)
    assert plan.capped
    assert plan.min_duration == pytest.approx(QUINTIC_PEAK_FACTOR * 1.0 / 2.0)


def test_the_peak_factor_stretches_a_move_that_is_under_the_cap_on_average():
    # 1.5 rad/s mean is under the 2 rad/s cap, but the quintic peaks at 2.8125.
    plan = plan_hand_speed(max_dist=1.5, grasp_time=1.0, cap=2.0)
    assert plan.speed == pytest.approx(1.5) and not plan.capped
    assert plan.min_duration == pytest.approx(QUINTIC_PEAK_FACTOR * 1.5 / 2.0)
    assert "longer than grasp_time" in describe_plan(plan, 1.0, 1.5)


@pytest.mark.parametrize(("max_dist", "grasp_time"), [(0.0, 1.0), (0.0005, 1.0), (1.0, 0.0)])
def test_no_usable_request_falls_back_and_still_respects_the_cap(max_dist, grasp_time):
    assert plan_hand_speed(max_dist, grasp_time, cap=5.0).speed == FALLBACK_SPEED
    low = plan_hand_speed(max_dist, grasp_time, cap=0.4)
    assert low.speed == pytest.approx(0.4)
    assert not low.capped  # nothing was asked for, so nothing was cut


@pytest.mark.parametrize("cap", [None, 0.0, -1.0, math.nan, math.inf])
def test_an_unreadable_cap_leaves_the_controller_the_judge(cap):
    plan = plan_hand_speed(max_dist=1.0, grasp_time=0.1, cap=cap)
    assert plan.speed == pytest.approx(10.0)
    assert not plan.capped


def test_the_plan_never_exceeds_the_cap_nor_promises_less_than_the_bound():
    for max_dist in (0.0, 0.01, 0.5, 2.3562, 4.7):
        for grasp_time in (0.0, 0.05, 0.5, 1.0, 3.0):
            for cap in (0.5, 2.0, 10.0):
                plan = plan_hand_speed(max_dist, grasp_time, cap)
                assert 0.0 < plan.speed <= cap
                assert plan.min_duration >= QUINTIC_PEAK_FACTOR * max_dist / cap - 1e-12
                assert plan.min_duration >= max_dist / plan.speed - 1e-12


def test_the_log_line_says_when_the_cap_cut_the_request():
    plan = plan_hand_speed(max_dist=1.0, grasp_time=0.2, cap=2.0)
    text = describe_plan(plan, 0.2, 1.0)
    assert "lowered" in text and "0.94" in text


# ── app.py wiring ────────────────────────────────────────────────────────────


def _gui():
    from integrated_bringup.demo_gui.app import DemoControllerGUI

    return DemoControllerGUI


def test_send_preset_no_longer_names_a_parameter_itself():
    source = inspect.getsource(_gui()._send_preset)
    assert "hand_trajectory_speed" not in source
    assert "_set_hand_speed_then(" in source


def test_the_hand_goal_is_published_from_the_set_completion():
    """`_publish_hand` sits in the callback handed to the speed set, not after
    the call — published at the click, the goal raced the set."""
    source = inspect.getsource(_gui()._send_preset)
    callback = source.index("def _send_hand_goal")
    assert callback < source.index("_publish_hand(") < source.index("_set_hand_speed_then(")
    assert source.count("_publish_hand(") == 1


def test_the_speed_set_reads_the_cap_first_and_marshals_to_tk():
    source = inspect.getsource(_gui()._set_hand_speed_then)
    assert source.index("get_parameters([cap_param])") > source.index("def _on_cap_done")
    assert "plan_hand_speed(max_dist, grasp_time, cap)" in source
    assert "set_parameters_atomically" in source
    # Both answers arrive on the executor thread.
    assert "self.root.after(0, _after_cap, cap)" in source
    assert "self.root.after(0, _after_set, plan, ok, reason)" in source
    # One completion, even when the services never answer.
    assert "PRESET_SPEED_TIMEOUT_MS" in source and 'state["done"]' in source


class _SpeedHarness:
    """`_set_hand_speed_then` against fake parameter services and a fake Tk
    `after` queue — the completion contract without a display."""

    def __init__(self, *, ready=True, cap=2.0, accept=True, answer=True):
        self.cap, self.accept, self.answer = cap, accept, answer
        self.sets: list[tuple[str, float]] = []
        self.shown: list[tuple[str, str, float]] = []
        self.logs: list[str] = []
        self.queue: list[tuple[int, object, tuple]] = []
        self.cancelled: list[object] = []
        self._ready = ready
        harness = self

        class _Future:
            def __init__(self, result):
                self._result = result

            def result(self):
                return self._result

            def add_done_callback(self, cb):
                if harness.answer:
                    cb(self)

        class _Value:
            type = 3
            double_value = cap

        class _Client:
            def services_are_ready(self):
                return harness._ready

            def get_parameters(self, names):
                values = [] if harness.cap is None else [_Value()]
                return _Future(type("R", (), {"values": values})())

            def set_parameters_atomically(self, params):
                harness.sets.append((params[0].name, params[0].value))
                result = type("S", (), {"successful": harness.accept, "reason": "no"})()
                return _Future(type("R", (), {"result": result})())

        class _Root:
            def after(self, ms, fn, *args):
                handle = object()
                harness.queue.append((ms, fn, args, handle))
                return handle

            def after_cancel(self, handle):
                harness.cancelled.append(handle)

        class _Logger:
            def __getattr__(self, _level):
                return harness.logs.append

        self.root = _Root()
        self._client = _Client()
        self._logger = _Logger()

    def get_logger(self):
        return self._logger

    def _get_param_client(self, _ctrl):
        return self._client

    def _show_param_value(self, ctrl, param, value):
        self.shown.append((ctrl, param, value))

    def run(self, ctrl="hand_joint_controller", max_dist=1.0, grasp_time=0.2):
        done: list[bool] = []
        _gui()._set_hand_speed_then(
            self, ctrl, max_dist, grasp_time, "Preset 'x'", lambda: done.append(True)
        )
        # Drain zero-delay callbacks (the executor → Tk hops), then the timer.
        while any(ms == 0 for ms, *_ in self.queue):
            i = next(i for i, (ms, *_) in enumerate(self.queue) if ms == 0)
            _ms, fn, args, _h = self.queue.pop(i)
            fn(*args)
        before_timer = len(done)
        for _ms, fn, args, handle in list(self.queue):
            if handle not in self.cancelled:
                fn(*args)
        return before_timer, len(done)


def test_set_is_capped_then_the_goal_goes_out_once():
    h = _SpeedHarness(cap=2.0)
    assert h.run(max_dist=1.0, grasp_time=0.2) == (1, 1)
    assert h.sets == [("trajectory_speed", pytest.approx(2.0))]
    assert ("hand_joint_controller", "trajectory_speed", pytest.approx(2.0)) in h.shown
    assert ("hand_joint_controller", "max_traj_velocity", 2.0) in h.shown
    assert len(h.cancelled) == 1  # the timeout was disarmed


def test_a_demo_controller_gets_its_own_spelling():
    h = _SpeedHarness(cap=4.0)
    assert h.run(ctrl="demo_joint_controller", max_dist=1.0, grasp_time=1.0) == (1, 1)
    assert h.sets == [("hand_trajectory_speed", pytest.approx(1.0))]


def test_a_refused_set_still_sends_the_goal_and_shows_no_speed():
    h = _SpeedHarness(accept=False)
    assert h.run() == (1, 1)
    assert not any(param == "trajectory_speed" for _c, param, _v in h.shown)
    assert any("refused" in line for line in h.logs)


def test_unready_services_send_the_goal_without_a_set():
    h = _SpeedHarness(ready=False)
    assert h.run() == (1, 1)
    assert h.sets == [] and h.queue == []


def test_a_silent_service_is_ended_by_the_timeout_exactly_once():
    h = _SpeedHarness(answer=False)
    assert h.run() == (0, 1)
    assert h.sets == []
    assert any("did not answer" in line for line in h.logs)


def test_a_controller_without_a_hand_speed_sends_the_goal_directly():
    h = _SpeedHarness()
    assert h.run(ctrl="demo_inference_controller") == (1, 1)
    assert h.sets == [] and h.queue == []


def test_an_unreadable_cap_sends_the_speed_uncapped():
    h = _SpeedHarness(cap=None)
    assert h.run(max_dist=1.0, grasp_time=0.1) == (1, 1)
    assert h.sets == [("trajectory_speed", pytest.approx(10.0))]
