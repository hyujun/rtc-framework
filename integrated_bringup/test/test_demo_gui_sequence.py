"""Headless tests for demo_controller_gui's posture sequence (Sequence tab).

What these tests hold:

- the step list: times on a 0.01 s grid, the file round trip, changing every
  step's time at once,
- the check before a run: all steps resolve to hand targets or nothing is sent,
- the run: a step ends on time AND arrival, once ends after one pass and loop
  goes round, and every reason a run ends on its own,
- the ``app.py`` wiring pytest can see: a step is published from the completion
  of the speed set, only the hand is commanded, nothing goes out after Stop,
  and the other senders are refused while a run plays.

Tk layout is checked at runtime, as for every other panel.
"""

from __future__ import annotations

import inspect
import json
import math
from types import MethodType, SimpleNamespace

import pytest

from integrated_bringup.demo_gui import sequence as seq
from integrated_bringup.demo_gui.app import SEQUENCE_TICK_MS, DemoControllerGUI
from integrated_bringup.demo_gui.sequence import (
    DWELLING,
    IDLE,
    MODE_LOOP,
    MODE_ONCE,
    MOVING,
    SENDING,
    ResolvedStep,
    SequenceFile,
    SequenceRunner,
    SequenceStep,
    load_sequence_file,
    moved,
    normalize_sequence_file,
    resolve_steps,
    save_sequence_file,
    step_problem,
    with_all_times,
)

_PRESETS = {
    "open": {"type": "open", "positions_deg": [0.0, 0.0], "grasp_time": 1.0},
    "close": {
        "type": "close",
        "positions_deg": [30.0, 60.0],
        "grasp_time": 2.0,
        "controller": "demo_task_controller",
        "robot_target": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
    },
    "ten_dof": {"type": "open", "positions_deg": [0.0] * 10, "grasp_time": 1.0},
}

# ── Step list ────────────────────────────────────────────────────────────────


def test_times_sit_on_the_hundredth_of_a_second_grid():
    step = SequenceStep("open", 0.126, 0.004)
    assert step.time_s == pytest.approx(0.13) and step.dwell_s == 0.0
    assert SequenceStep("open", 1.005, 0.015).dwell_s in (0.01, 0.02)
    # 0.07 is not representable; the stored value is still exactly the 2-digit one.
    assert repr(SequenceStep("open", 0.07).time_s) == "0.07"


def test_a_time_below_the_grid_is_raised_to_it_and_a_dwell_may_be_zero():
    step = SequenceStep("open", 0.0, -1.0)
    assert step.time_s == seq.MIN_TIME_S
    assert step.dwell_s == 0.0
    assert SequenceStep("open", 1e9).time_s == seq.MAX_TIME_S


def test_every_step_takes_the_new_time_at_once():
    steps = (SequenceStep("open", 1.0, 0.5), SequenceStep("close", 2.0, 0.0))
    timed = with_all_times(steps, time_s=0.25)
    assert [s.time_s for s in timed] == [0.25, 0.25]
    assert [s.dwell_s for s in timed] == [0.5, 0.0]  # dwell untouched
    assert [s.preset for s in timed] == ["open", "close"]
    dwelled = with_all_times(steps, dwell_s=0.1)
    assert [s.time_s for s in dwelled] == [1.0, 2.0]
    assert [s.dwell_s for s in dwelled] == [0.1, 0.1]
    assert with_all_times(steps, time_s=0.001)[0].time_s == seq.MIN_TIME_S


def test_moving_a_row_stops_at_the_ends():
    steps = tuple(SequenceStep(name, 1.0) for name in "abc")
    rows, at = moved(steps, 0, 1)
    assert [s.preset for s in rows] == ["b", "a", "c"] and at == 1
    rows, at = moved(steps, 0, -1)
    assert [s.preset for s in rows] == ["a", "b", "c"] and at == 0
    rows, at = moved(steps, 2, 1)
    assert [s.preset for s in rows] == ["a", "b", "c"] and at == 2


def test_the_file_round_trips(tmp_path):
    path = str(tmp_path / "nested" / "demo_gui_sequences_p2.json")
    stored = SequenceFile(
        (SequenceStep("open", 0.25, 0.1), SequenceStep("open", 1.5)), tolerance_deg=3.5
    )
    save_sequence_file(path, stored)
    assert load_sequence_file(path) == stored
    with open(path) as stored_file:
        raw = json.load(stored_file)
    assert raw["steps"][0] == {"preset": "open", "time_s": 0.25, "dwell_s": 0.1}
    assert [p.name for p in (tmp_path / "nested").iterdir()] == ["demo_gui_sequences_p2.json"]


def test_a_missing_or_broken_file_is_an_empty_list(tmp_path):
    assert load_sequence_file(str(tmp_path / "absent.json")) == SequenceFile()
    broken = tmp_path / "broken.json"
    broken.write_text("{not json")
    assert load_sequence_file(str(broken)) == SequenceFile()


def test_malformed_steps_are_dropped_not_guessed():
    data = {
        "steps": [
            {"preset": "open", "time_s": 1.0},
            {"preset": "", "time_s": 1.0},
            {"preset": "open", "time_s": "fast"},
            {"preset": "open", "time_s": float("nan")},
            {"preset": "open", "time_s": True},
            "open",
            {"preset": "close", "time_s": 2, "dwell_s": 0.5},
        ],
        "tolerance_deg": "wide",
    }
    loaded = normalize_sequence_file(data)
    assert loaded.steps == (SequenceStep("open", 1.0), SequenceStep("close", 2.0, 0.5))
    assert loaded.tolerance_deg == seq.DEFAULT_TOLERANCE_DEG
    assert normalize_sequence_file([]) == SequenceFile()
    assert normalize_sequence_file({"tolerance_deg": 500}).tolerance_deg == seq.MAX_TOLERANCE_DEG


# ── Check before a run ───────────────────────────────────────────────────────


def test_steps_resolve_to_the_presets_hand_targets_only():
    steps = (SequenceStep("open", 0.5), SequenceStep("close", 0.75, 0.2))
    resolved, problem = resolve_steps(steps, _PRESETS, 2)
    assert problem == ""
    assert resolved[1] == ResolvedStep(
        "close", (math.radians(30.0), math.radians(60.0)), 0.75, 0.2
    )
    # The step's own time, not the preset's grasp_time; no arm, no controller.
    assert {f.name for f in resolved[1].__dataclass_fields__.values()} == {
        "preset",
        "target_rad",
        "time_s",
        "dwell_s",
    }


@pytest.mark.parametrize(
    ("steps", "expected"),
    [
        ((), "empty"),
        ((SequenceStep("open", 1.0), SequenceStep("gone", 1.0)), "step 2 'gone'"),
        ((SequenceStep("ten_dof", 1.0),), "hand DoF (2)"),
    ],
)
def test_one_unusable_step_refuses_the_whole_list(steps, expected):
    resolved, problem = resolve_steps(steps, _PRESETS, 2)
    assert resolved == () and expected in problem


def test_a_non_numeric_position_is_a_problem():
    presets = {"bad": {"positions_deg": [0.0, "x"]}}
    assert "non-numeric" in step_problem(SequenceStep("bad", 1.0), presets, 2)
    assert step_problem(SequenceStep("open", 1.0), _PRESETS, 2) == ""


def test_a_run_plays_its_snapshot_not_the_live_presets():
    presets = {"a": {"positions_deg": [10.0, 10.0]}}
    resolved, _ = resolve_steps((SequenceStep("a", 1.0),), presets, 2)
    presets["a"]["positions_deg"] = [90.0, 90.0]
    del presets["a"]
    assert resolved[0].target_rad == pytest.approx((math.radians(10.0),) * 2)


# ── The run ──────────────────────────────────────────────────────────────────

_A = ResolvedStep("a", (0.0, 0.0), 0.5, 0.0)
_B = ResolvedStep("b", (1.0, 1.0), 0.25, 0.2)


def _started(steps=(_A, _B), mode=MODE_ONCE, tolerance=2.0) -> SequenceRunner:
    runner = SequenceRunner()
    assert runner.start(steps, mode, tolerance, ("thumb", "index")) == ""
    return runner


def _send(runner, now, measured=(0.0, 0.0), min_duration=None):
    """Tick until a step is handed out, acknowledge it as published at ``now``."""
    step = runner.tick(now, measured, 0.0)
    assert step is not None and runner.state == SENDING
    runner.on_sent(runner.token, now, "", min_duration)
    assert runner.state == MOVING
    return step


def test_start_refuses_an_empty_list_a_bad_mode_and_a_second_run():
    runner = SequenceRunner()
    assert "empty" in runner.start((), MODE_ONCE)
    assert "unknown mode" in runner.start((_A,), "twice")
    assert not runner.running
    assert runner.start((_A,), MODE_ONCE) == ""
    assert "already playing" in runner.start((_A,), MODE_ONCE)


def test_a_step_ends_only_when_its_time_has_passed_and_the_hand_arrived():
    runner = _started()
    assert _send(runner, 10.0) is _A
    # Arrived (the hand already was there) but the time is not over.
    assert runner.tick(10.49, (0.0, 0.0), 0.0) is None and runner.state == MOVING
    # Time over, hand not there.
    assert runner.tick(10.6, (0.5, 0.0), 0.0) is None and runner.state == MOVING
    # Both: the next step goes out in the same tick.
    assert runner.tick(10.7, (0.0, 0.01), 0.0) is _B
    assert runner.index == 1 and runner.state == SENDING


def test_the_time_counts_from_the_publish_not_from_the_hand_out():
    runner = _started()
    assert runner.tick(10.0, (0.0, 0.0), 0.0) is _A
    runner.on_sent(runner.token, 10.4, "")  # the speed set took 0.4 s
    assert runner.tick(10.6, (0.0, 0.0), 0.0) is None  # 0.5 s from 10.0, 0.2 s from the publish
    assert runner.tick(10.9, (0.0, 0.0), 0.0) is _B


def test_a_capped_move_waits_for_the_plans_lower_bound():
    runner = _started()
    _send(runner, 0.0, min_duration=1.2)
    assert runner.expected_s == pytest.approx(1.2)
    assert runner.tick(0.6, (0.0, 0.0), 0.0) is None  # time_s 0.5 is over, the plan is not
    assert runner.tick(1.21, (0.0, 0.0), 0.0) is _B


def test_the_dwell_is_held_after_arrival():
    runner = _started()
    _send(runner, 0.0)
    assert runner.tick(0.5, (0.0, 0.0), 0.0) is _B
    runner.on_sent(runner.token, 0.5)
    assert runner.tick(0.8, (1.0, 1.0), 0.0) is None and runner.state == DWELLING
    assert runner.tick(0.99, (1.0, 1.0), 0.0) is None and runner.running
    assert runner.tick(1.01, (1.0, 1.0), 0.0) is None
    assert not runner.running


def test_once_ends_after_one_pass_on_the_last_step():
    runner = _started()
    _send(runner, 0.0)
    runner.tick(0.5, (0.0, 0.0), 0.0)
    runner.on_sent(runner.token, 0.5)
    runner.tick(0.8, (1.0, 1.0), 0.0)
    runner.tick(1.1, (1.0, 1.0), 0.0)
    assert runner.state == IDLE and not runner.aborted
    assert runner.cycle == 1 and runner.index == 1
    assert "done" in runner.result
    assert runner.status(2.0) == runner.result
    assert runner.tick(5.0, (1.0, 1.0), 0.0) is None  # nothing after the end


def test_loop_goes_round_until_stopped():
    steps = (ResolvedStep("a", (0.0,), 0.1, 0.0), ResolvedStep("b", (1.0,), 0.1, 0.0))
    runner = _started(steps, MODE_LOOP)
    now, sent = 0.0, []
    step = runner.tick(now, (0.0,), 0.0)
    for _ in range(7):
        sent.append(step.preset)
        runner.on_sent(runner.token, now)
        now += 0.11
        step = runner.tick(now, step.target_rad, 0.0)
        assert step is not None
    assert sent == ["a", "b", "a", "b", "a", "b", "a"]
    assert runner.cycle == 3 and runner.running
    runner.stop()
    assert not runner.running and not runner.aborted
    assert "stopped by the operator" in runner.result and "step 2/2 'b'" in runner.result


def test_nothing_is_sent_after_stop_even_for_a_send_in_flight():
    runner = _started()
    assert runner.tick(0.0, (0.0, 0.0), 0.0) is _A
    token = runner.token
    assert runner.awaiting(token)
    runner.stop()
    assert not runner.awaiting(token)
    runner.on_sent(token, 0.1)  # the late completion changes nothing
    assert runner.state == IDLE and "stopped" in runner.result


def test_a_completion_of_an_earlier_run_is_not_taken_for_this_one():
    runner = _started()
    runner.tick(0.0, (0.0, 0.0), 0.0)
    stale = runner.token
    runner.stop()
    assert runner.start((_A, _B), MODE_ONCE) == ""
    runner.tick(1.0, (0.0, 0.0), 0.0)
    assert not runner.awaiting(stale)
    runner.on_sent(stale, 1.0)
    assert runner.state == SENDING


# ── Every way a run ends on its own ──────────────────────────────────────────


@pytest.mark.parametrize("state", ["sending", "moving", "dwelling"])
def test_a_block_ends_the_run_in_any_state(state):
    runner = _started((_B, _A))
    runner.tick(0.0, (0.0, 0.0), 0.0)
    if state != "sending":
        runner.on_sent(runner.token, 0.0)
    if state == "dwelling":
        runner.tick(0.3, (1.0, 1.0), 0.0)
    assert runner.state == state
    assert runner.tick(0.31, (1.0, 1.0), 0.0, "E-STOP is active") is None
    assert runner.state == IDLE and runner.aborted
    assert "E-STOP is active" in runner.result and "step 1/2 'b'" in runner.result


def test_a_refused_send_ends_the_run_with_the_reason():
    runner = _started()
    runner.tick(0.0, (0.0, 0.0), 0.0)
    runner.on_sent(runner.token, 0.0, "hand_joint_controller mode is damping")
    assert runner.aborted and "was not sent — hand_joint_controller mode is damping" in (
        runner.result
    )


def test_a_send_that_never_completes_ends_the_run():
    runner = _started()
    runner.tick(0.0, (0.0, 0.0), 0.0)
    assert runner.tick(seq.SEND_TIMEOUT_S, (0.0, 0.0), 0.0) is None and runner.running
    runner.tick(seq.SEND_TIMEOUT_S + 0.01, (0.0, 0.0), 0.0)
    assert runner.aborted and "never completed" in runner.result


def test_an_unreached_target_times_out_naming_the_joint():
    """A joint held short of its target (a clamp, an end stop) must end the
    run, not let the next step go out on top of it."""
    runner = _started((_B,))
    _send(runner, 0.0)
    short = (1.0, 1.0 - math.radians(6.2))
    timeout = seq.TIMEOUT_FACTOR * 0.25 + seq.TIMEOUT_MARGIN_S
    assert runner.tick(timeout, short, 0.0) is None and runner.running
    runner.tick(timeout + 0.01, short, 0.0)
    assert runner.aborted
    assert "index is 6.2 deg off (tolerance 2.0 deg)" in runner.result


def test_the_tolerance_decides_what_reached_means():
    short = (1.0, 1.0 - math.radians(6.2))
    strict = _started((_B, _A), tolerance=2.0)
    _send(strict, 0.0)
    assert strict.tick(0.3, short, 0.0) is None and strict.state == MOVING
    loose = _started((_B, _A), tolerance=7.0)
    _send(loose, 0.0)
    assert loose.tick(0.3, short, 0.0) is None and loose.state == DWELLING


@pytest.mark.parametrize(
    ("age", "expected"),
    [(None, "no hand joint state was ever received"), (1.5, "stale (last one 1.5 s ago)")],
)
def test_a_stale_measurement_is_not_an_arrival(age, expected):
    runner = _started((_A, _B))
    _send(runner, 0.0)
    # The frozen value is exactly the target — and still no arrival.
    assert runner.tick(0.6, (0.0, 0.0), age) is None and runner.state == MOVING
    runner.tick(seq.TIMEOUT_FACTOR * 0.5 + seq.TIMEOUT_MARGIN_S + 0.01, (0.0, 0.0), age)
    assert runner.aborted and expected in runner.result


def test_a_measurement_of_another_size_is_not_an_arrival():
    runner = _started((_A, _B))
    _send(runner, 0.0)
    assert runner.tick(0.6, (0.0,), 0.0) is None
    runner.tick(10.0, (0.0,), 0.0)
    assert runner.aborted and "does not match the target's size" in runner.result


def test_the_status_line_names_cycle_step_and_phase():
    assert SequenceRunner().status(0.0) == "idle"
    runner = _started(mode=MODE_LOOP)
    _send(runner, 0.0)
    assert runner.status(0.3) == "playing (loop) — cycle 1, step 1/2 'a': moving 0.3 / 0.50 s"


# ── app.py wiring ────────────────────────────────────────────────────────────


def test_the_run_loop_is_not_coarser_than_the_time_grid():
    assert SEQUENCE_TICK_MS / 1000.0 <= seq.TIME_RESOLUTION_S


def test_a_step_is_published_from_the_set_completion_and_commands_no_arm():
    source = inspect.getsource(DemoControllerGUI._send_sequence_step)
    callback = source.index("def _send_step_goal")
    assert callback < source.index("_publish_hand(") < source.index("_set_hand_speed_then(")
    assert source.count("_publish_hand(") == 1
    # Hand only: neither the arm publisher nor a controller switch.
    for forbidden in ("robot_cmd_pub", "_on_switch_controller", "_send_preset", "selected_ctrl"):
        assert forbidden not in source


def test_the_other_senders_are_refused_while_a_run_plays():
    gui = inspect.getsource(DemoControllerGUI._run_gui)
    assert 'self._unless_sequence(self._publish_target, "Send Command")' in gui
    grasp = inspect.getsource(DemoControllerGUI._build_grasp_tab)
    assert 'self._unless_sequence(self._send_preset, "Send Preset")' in grasp
    assert "_unless_sequence(" in inspect.getsource(DemoControllerGUI._build_hand_step_panel)
    assert "self._toggle_press_unless_sequence" in gui


def _key(widget_class="TButton"):
    return SimpleNamespace(
        keysym="space", widget=SimpleNamespace(winfo_class=lambda: widget_class)
    )


def _toggle_gui(running: bool):
    logger, passed = _Logger(), []
    gui = SimpleNamespace(
        _sequence_runner=SimpleNamespace(running=running),
        _sequence_toggle_warned=False,
        _toggle_capture_window=None,
        _toggle_event_matches=lambda event: event.keysym == "space",
        _on_toggle_key_press=lambda event: passed.append(event) or "passed",
        get_logger=lambda: logger,
    )
    return gui, logger, passed


def test_the_toggle_shortcut_is_swallowed_while_a_run_plays():
    gui, logger, passed = _toggle_gui(running=True)
    press = DemoControllerGUI._toggle_press_unless_sequence
    assert press(gui, _key()) == "break"
    assert press(gui, _key()) == "break"  # auto-repeat
    assert passed == [] and len(logger.lines) == 1
    # Typing a space into a text field is not the shortcut.
    assert press(gui, _key("TEntry")) == "passed"


def test_the_toggle_shortcut_is_untouched_when_nothing_plays():
    gui, logger, passed = _toggle_gui(running=False)
    assert DemoControllerGUI._toggle_press_unless_sequence(gui, _key()) == "passed"
    assert len(passed) == 1 and logger.lines == []


class _Logger:
    def __init__(self):
        self.lines: list[str] = []

    def __getattr__(self, _level):
        return self.lines.append


def _fake_gui(**overrides):
    """What the sequence methods read of the GUI, without Tk or rclpy."""
    published: list = []
    logger = _Logger()
    gui = SimpleNamespace(
        _sequence_runner=SequenceRunner(),
        _sequence_ctrl="hand_joint_controller",
        _active_ctrl="hand_joint_controller",
        estop_active=False,
        current_hand_positions=[0.0, 0.0],
        _shape=SimpleNamespace(hand_motor_names=("thumb", "index"), hand_dof=2),
        hand_cmd_pub=SimpleNamespace(publish=published.append),
        get_logger=lambda: logger,
        published=published,
        logger=logger,
        speed_calls=[],
        mode_block="",
    )
    gui._hand_goal_block = lambda: gui.mode_block

    def _publish_hand(msg, what):
        if gui.mode_block:
            return gui.mode_block
        gui.hand_cmd_pub.publish(msg)
        return ""

    def _set_hand_speed_then(ctrl, max_dist, grasp_time, what, then, on_plan=None):
        gui.speed_calls.append(
            SimpleNamespace(
                ctrl=ctrl, max_dist=max_dist, grasp_time=grasp_time, then=then, on_plan=on_plan
            )
        )

    gui._publish_hand = _publish_hand
    gui._set_hand_speed_then = _set_hand_speed_then
    for name in ("_sequence_run_block", "_send_sequence_step", "_unless_sequence"):
        setattr(gui, name, MethodType(getattr(DemoControllerGUI, name), gui))
    for key, value in overrides.items():
        setattr(gui, key, value)
    return gui


def _hand_out(gui, steps=(_B, _A)):
    runner = gui._sequence_runner
    assert runner.start(steps, MODE_ONCE) == ""
    step = runner.tick(0.0, gui.current_hand_positions, 0.0)
    gui._send_sequence_step(step, runner.token)
    assert len(gui.speed_calls) == 1
    return gui.speed_calls[0]


def test_a_step_sets_the_speed_for_its_own_time_then_publishes_the_hand_target():
    gui = _fake_gui()
    call = _hand_out(gui)
    assert (call.ctrl, call.grasp_time) == ("hand_joint_controller", 0.25)
    assert call.max_dist == pytest.approx(1.0)
    assert gui.published == []  # not before the set has answered
    call.on_plan(SimpleNamespace(min_duration=0.9))
    call.then()
    assert len(gui.published) == 1
    msg = gui.published[0]
    assert msg.goal_type == "joint"
    assert list(msg.joint_names) == ["thumb", "index"]
    assert list(msg.joint_target) == pytest.approx([1.0, 1.0])
    runner = gui._sequence_runner
    assert runner.state == MOVING and runner.expected_s == pytest.approx(0.9)


def test_without_a_plan_the_step_runs_on_its_own_time():
    gui = _fake_gui()
    _hand_out(gui).then()
    assert gui._sequence_runner.expected_s == pytest.approx(0.25)


def test_a_stop_during_the_speed_set_publishes_nothing():
    gui = _fake_gui()
    call = _hand_out(gui)
    gui._sequence_runner.stop()
    call.then()
    assert gui.published == []
    assert "stopped by the operator" in gui._sequence_runner.result


@pytest.mark.parametrize(
    ("change", "expected"),
    [
        ({"estop_active": True}, "E-STOP is active"),
        ({"_active_ctrl": "demo_joint_controller"}, "changed from hand_joint_controller"),
        ({"mode_block": "mode is damping"}, "mode is damping"),
        ({"hand_cmd_pub": None}, "not bound"),
    ],
)
def test_what_changed_during_the_speed_set_withholds_the_step(change, expected):
    gui = _fake_gui()
    call = _hand_out(gui)
    for key, value in change.items():
        setattr(gui, key, value)
    call.then()
    assert gui.published == []
    runner = gui._sequence_runner
    assert runner.aborted and expected in runner.result


def test_a_button_is_refused_while_playing_and_works_otherwise():
    gui = _fake_gui()
    calls: list[int] = []
    command = gui._unless_sequence(lambda: calls.append(1), "Send Command")
    command()
    assert calls == [1]
    gui._sequence_runner.start((_A,), MODE_LOOP)
    command()
    assert calls == [1]
    assert any("Send Command ignored" in line for line in gui.logger.lines)


def test_the_tick_unlocks_the_tab_when_it_raises():
    source = inspect.getsource(DemoControllerGUI._sequence_tick)
    handler = source.index("except Exception")
    assert handler < source.index("runner.stop(", handler) < source.index("raise", handler)
    assert "self._sequence_finished()" in source[handler:]
    # The repaint and the reschedule are inside the guarded part too.
    reschedule = source.index("self.root.after(SEQUENCE_TICK_MS, self._sequence_tick)")
    assert source.index("self._paint_sequence()") < reschedule < handler


def test_play_cancels_a_leftover_tick_before_starting_its_own():
    source = inspect.getsource(DemoControllerGUI._play_sequence)
    assert source.index("after_cancel(self._sequence_after)") < source.index(
        "self._sequence_tick()"
    )


class _OneSeverityPerCallSite:
    """rclpy's rule: a logging call site may not change severity between calls."""

    def __init__(self):
        self.sites: dict[tuple, str] = {}
        self.lines: list[tuple[str, str]] = []

    def _log(self, level, text):
        frame = inspect.stack()[2]
        site = (frame.filename, frame.lineno)
        assert self.sites.setdefault(site, level) == level, "severity changed at one call site"
        self.lines.append((level, text))

    def info(self, text):
        self._log("info", text)

    def warn(self, text):
        self._log("warn", text)


def test_the_end_of_a_run_unlocks_the_tab_and_logs_at_either_severity():
    """An aborted run after a finished one used to raise in rclpy's logger
    (one call site, two severities) before the tab was unlocked."""
    logger, order = _OneSeverityPerCallSite(), []
    gui = SimpleNamespace(
        _sequence_runner=SimpleNamespace(result="done", aborted=False, cycle=1),
        _apply_sequence_lock=lambda: order.append("unlock"),
        _paint_sequence=lambda ok=None: order.append(f"paint {ok}"),
        get_logger=lambda: (order.append("log"), logger)[1],
    )
    DemoControllerGUI._sequence_finished(gui)
    gui._sequence_runner = SimpleNamespace(result="aborted — x", aborted=True, cycle=0)
    DemoControllerGUI._sequence_finished(gui)
    assert [level for level, _ in logger.lines] == ["info", "warn"]
    assert order[:3] == ["unlock", "paint True", "log"]
    assert gui._sequence_message == "aborted — x"
