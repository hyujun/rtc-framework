"""Unit tests for demo_gui.task_frame — active-controller task-frame selection (#292).

The GUI used to read a profile-fixed ``tool0_actual`` while the task controller
may actually be controlling a virtual TCP, so its "current pose" and the goal it
published were in different frames: copying the displayed pose into the target
box commanded a real motion instead of a no-op.

Most of this file drives ``TaskFrameSelector`` directly — it is pure Python, so
the whole state machine is testable without a display or a ROS graph (following
``test_demo_gui_pull``). The last group asserts against ``app.py`` source, which
is where the wiring lives: the selector is only useful if the rewire path
invalidates it and nothing seeds the task panel behind its back.
"""

from __future__ import annotations

import math

import pytest

from integrated_bringup.demo_gui.task_frame import (
    VIRTUAL_TCP_FRAME,
    TaskFrameSelector,
)

FALLBACK = "tool0_actual"

# What one controller tick's TFMessage carries, by phase. During a closed-chain
# hand's FK walk-in the controller publishes its arm tip and fingertips but no
# virtual TCP, because owned_topics.cpp gates that slot on
# virtual_tcp_pose_valid — which is exactly the case the settle window exists
# for.
WALKIN_FRAMES = [FALLBACK, "thumb_tip_link_actual", "index_tip_link_actual"]
SETTLED_FRAMES = [*WALKIN_FRAMES, VIRTUAL_TCP_FRAME]


def _observe_n(sel: TaskFrameSelector, frames: list[str], n: int) -> None:
    for _ in range(n):
        sel.observe(frames)


# ── Selection ────────────────────────────────────────────────────────────────


def test_virtual_tcp_latches_on_first_sighting():
    """The controller only broadcasts the virtual TCP on ticks it is actually
    controlling that point, so one sighting is conclusive — no window needed."""
    sel = TaskFrameSelector(FALLBACK)
    assert sel.selection() is None

    sel.observe(SETTLED_FRAMES)
    assert sel.selection() == VIRTUAL_TCP_FRAME


def test_fallback_latches_only_after_the_settle_window():
    """A controller that never publishes a virtual TCP settles on tool0 — but
    not before the window, or the walk-in case below would break."""
    sel = TaskFrameSelector(FALLBACK, settle_msgs=5)

    _observe_n(sel, WALKIN_FRAMES, 4)
    assert sel.selection() is None, "latched before the window elapsed"

    sel.observe(WALKIN_FRAMES)
    assert sel.selection() == FALLBACK


def test_virtual_tcp_arriving_inside_the_window_wins():
    """The walk-in case, and the reason the window exists at all.

    A vtcp-configured task controller publishes tool0 and nothing else while its
    closed-chain hand FK converges. Latching on the first message would pick
    tool0 — precisely the wrong frame in precisely the situation this module was
    written for.
    """
    sel = TaskFrameSelector(FALLBACK, settle_msgs=15)

    _observe_n(sel, WALKIN_FRAMES, 14)
    assert sel.selection() is None

    sel.observe(SETTLED_FRAMES)
    assert sel.selection() == VIRTUAL_TCP_FRAME


def test_selection_is_frozen_once_latched():
    """A frame that drops out for a tick must not move the readout to another
    control point — the C++ side holds the arm through exactly that flicker, and
    a GUI that hopped frames with it would report motion that is not happening.
    """
    sel = TaskFrameSelector(FALLBACK, settle_msgs=2)
    sel.observe(SETTLED_FRAMES)
    assert sel.selection() == VIRTUAL_TCP_FRAME

    _observe_n(sel, WALKIN_FRAMES, 50)
    assert sel.selection() == VIRTUAL_TCP_FRAME

    # And the converse: a settled fallback is not upgraded mid-session either.
    sel2 = TaskFrameSelector(FALLBACK, settle_msgs=2)
    _observe_n(sel2, WALKIN_FRAMES, 2)
    assert sel2.selection() == FALLBACK
    sel2.observe(SETTLED_FRAMES)
    assert sel2.selection() == FALLBACK


def test_messages_carrying_neither_frame_are_not_evidence():
    """Such a message says nothing about the task frame, so it must not advance
    the window toward a fallback the controller never published."""
    sel = TaskFrameSelector(FALLBACK, settle_msgs=3)
    _observe_n(sel, ["some_other_link_actual"], 10)
    assert sel.selection() is None


def test_on_rewire_invalidates_everything():
    """Nothing observed under the previous controller is evidence about the next
    one, which may control a different point entirely."""
    sel = TaskFrameSelector(FALLBACK, settle_msgs=2)
    sel.observe(SETTLED_FRAMES)
    sel.mark_seeded()
    assert sel.selection() == VIRTUAL_TCP_FRAME
    assert sel.seeded() is True

    sel.on_rewire()
    assert sel.selection() is None
    assert sel.seeded() is False

    # The window restarts too — a partially-elapsed count does not carry over.
    sel.observe(WALKIN_FRAMES)
    assert sel.selection() is None
    sel.observe(WALKIN_FRAMES)
    assert sel.selection() == FALLBACK


def test_seeded_flag_is_one_shot_per_rewire():
    """The task target panel is seeded exactly once after the frame settles;
    re-seeding every frame would fight the operator's typing."""
    sel = TaskFrameSelector(FALLBACK)
    assert sel.seeded() is False
    sel.mark_seeded()
    assert sel.seeded() is True
    sel.mark_seeded()
    assert sel.seeded() is True


# ── app.py wiring ────────────────────────────────────────────────────────────


def test_rewire_replaces_the_tf_buffer_and_invalidates_the_frame():
    """tf2 caches transforms for 10 s by default, so without replacing the
    buffer the PREVIOUS controller's virtual_tcp_actual keeps resolving through
    lookup_transform after its publisher is gone. The availability test would
    then answer for the wrong controller and feed its stale pose to the readout.

    Driven against the reset helper directly — the surrounding rewire destroys
    and recreates rclpy handles, which needs a live node (the AC-5 pattern from
    test_demo_gui_pull).
    """
    from integrated_bringup.demo_gui.app import DemoControllerGUI
    from integrated_bringup.demo_gui.pull import PULL_UNAVAILABLE, PullPeakHold, PullSnapshot

    class _State:
        pass

    state = _State()
    state._pull = PullSnapshot.unavailable()
    state._pull_peak = PullPeakHold()
    state._wbc_active = False
    state._task_frame = TaskFrameSelector(FALLBACK)
    state._task_frame.observe(SETTLED_FRAMES)
    state._task_frame.mark_seeded()
    state._task_pose_valid = True
    old_buffer = object()
    state._tf_buffer = old_buffer

    DemoControllerGUI._reset_controller_sourced_state(state)

    assert state._tf_buffer is not old_buffer, "stale tf cache survived the rewire"
    assert state._task_frame.selection() is None
    assert state._task_frame.seeded() is False
    assert state._task_pose_valid is False
    assert state._pull is PULL_UNAVAILABLE


def test_switch_controller_does_not_seed_the_task_panel():
    """At switch time the new controller has not published a transform yet, so
    current_task_positions still holds the PREVIOUS controller's pose. Seeding
    the target box from it is how "copy the displayed pose and the robot moves"
    happened. The Tk refresh seeds it once the frame settles instead.
    """
    import inspect

    from integrated_bringup.demo_gui.app import DemoControllerGUI

    source = inspect.getsource(DemoControllerGUI._on_switch_controller)
    assert "_set_task_target_entries" not in source
    # The joint panel is unaffected — it has no frame ambiguity.
    assert "_set_joint_target_entries" in source

    refresh = inspect.getsource(DemoControllerGUI._refresh_current_display)
    assert "_set_task_target_entries" in refresh
    assert "_task_frame.seeded()" in refresh
    assert "mark_seeded()" in refresh


def test_every_task_goal_egress_passes_the_frame_gate():
    """The three routes that can put a task goal on the wire. _send_preset is
    the sharp one: it publishes automatically right after switching controllers,
    so it can author a goal inside the very window the gate exists to close.
    """
    import inspect

    from integrated_bringup.demo_gui.app import DemoControllerGUI

    for method in (
        DemoControllerGUI._publish_target,
        DemoControllerGUI._send_preset,
        DemoControllerGUI._copy_current_to_target,
    ):
        assert "_task_space_ready" in inspect.getsource(method), method.__name__

    gate = inspect.getsource(DemoControllerGUI._task_space_ready)
    assert "selection() is not None" in gate


def test_task_pose_lookup_uses_the_selected_frame():
    """The lookup must follow the selector, not the profile's fixed child frame,
    and must withhold the pose entirely while the frame is unsettled."""
    import inspect

    from integrated_bringup.demo_gui.app import DemoControllerGUI

    source = inspect.getsource(DemoControllerGUI._arm_joint_cb)
    assert "_task_frame.selection()" in source
    assert "_tf_child_frame" not in source, "still reading the profile-fixed frame"
    assert "_task_pose_valid = False" in source


# ── Unit conventions (unchanged by #292) ─────────────────────────────────────


def test_task_entry_degree_round_trip_is_identity():
    """The task panel writes orientation entries in degrees
    (_set_task_target_entries) and _publish_target converts them back with
    math.radians. #292 moved WHEN those run, never the conversion — a regression
    here would silently scale every orientation goal.
    """
    for rad in (0.0, 0.5, -1.5708, 3.0, -3.14159):
        text = f"{math.degrees(rad):.4f}"  # what _set_task_target_entries writes
        assert math.radians(float(text)) == pytest.approx(rad, abs=1e-6)

    for metres in (0.0, 0.4137, -0.0866):
        text = f"{metres:.4f}"  # translation entries are metres on both sides
        assert float(text) == pytest.approx(metres, abs=1e-6)


