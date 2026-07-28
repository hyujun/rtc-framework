"""Active-controller task-frame selection for demo_controller_gui (#292).

The task controller does not always control the same point. With
``virtual_tcp_mode`` enabled it controls a virtual TCP derived from fingertip
kinematics; it falls back to tool0 on any tick the hand FK produces nothing,
which is every tick of a closed-chain hand's FK walk-in. The GUI read a
profile-fixed ``tool0_actual`` regardless, so the pose it displayed as "current"
belonged to a different frame than the goal it published — and copying the
displayed pose into the target box commanded a real motion instead of a no-op.

The frame is chosen from **availability**, never from a controller name. The
controller publishes ``virtual_tcp_actual`` only on ticks where it is actually
controlling that point (``owned_topics.cpp`` gates the slot on
``virtual_tcp_pose_valid``), so observing the frame at all is the evidence. A
controller that never publishes it — WBC, or a task controller with the vtcp
disabled — simply settles on the fallback, and no list of controller names has
to be kept in sync here.

Why a settle window instead of "first message wins": during the walk-in a
vtcp-configured task controller publishes tool0 and nothing else. Latching on
the first message would pick precisely the wrong frame in precisely the case
this module exists for, so the fallback has to out-wait the walk-in. Once
latched, the selection is frozen until the next rewire: a frame that drops out
for a tick must not make the GUI's readout hop between control points.

Pure Python — no Tk, no rclpy — so this is unit-testable without a display or a
ROS graph, following ``demo_gui.pull`` / ``demo_gui.catalog``.

Public surface (imported by app.py):
- VIRTUAL_TCP_FRAME
- TaskFrameSelector
"""

from __future__ import annotations

import threading
from collections.abc import Iterable
from dataclasses import dataclass, replace

# Child frame the controllers broadcast for the virtual TCP control point.
# Owner: integrated_bringup/src/support/owned_topics.cpp (AppendVirtualTcpSlot).
VIRTUAL_TCP_FRAME = "virtual_tcp_actual"

# TFMessages carrying the fallback frame but no virtual TCP that must arrive
# before the fallback is latched.
#
# The window has to outlast the closed-chain hand FK walk-in, because a
# vtcp-configured controller publishes tool0 alone for its whole duration and
# latching tool0 there is the exact error this module exists to prevent. A
# measured p1b walk-in is ~93 RT ticks, and the controller emits at most one
# TFMessage per tick (the CM's publish lane coalesces, never duplicates) — so a
# window of N messages always spans at least N ticks, and 150 clears 93 with
# margin regardless of the wire rate. Counting messages rather than seconds is
# what makes that argument hold: the walk-in is ⌈Δ/max_seed_increment⌉
# iterations long, so it scales with ticks, not with wall time (the same reason
# the C++ side's kVtcpFrameWaitTicks is a tick count).
#
# The cost of overshooting is small and self-correcting: a controller that never
# publishes a virtual TCP leaves the task panel disabled for this many messages
# — a fraction of a second at any realistic rate — and a rewire resets it.
DEFAULT_SETTLE_MSGS = 150


@dataclass(frozen=True)
class _State:
    """One immutable snapshot of the selector's state.

    Rebound as a single attribute so a reader on the Tk thread can never observe
    a half-updated selection (the same discipline as ``PullSnapshot``).
    """

    selection: str | None = None
    fallback_seen: int = 0
    seeded: bool = False


class TaskFrameSelector:
    """Pick the TF child frame the active controller actually controls in.

    Written from the rclpy executor thread (``observe``, ``on_rewire``) and read
    from the Tk thread (``selection``, ``seeded``); ``mark_seeded`` is written
    from Tk. Writers take a lock so a concurrent ``observe`` cannot lose a
    ``mark_seeded``; readers take none, because every write publishes a whole
    new snapshot.
    """

    def __init__(self, fallback_child: str, settle_msgs: int = DEFAULT_SETTLE_MSGS) -> None:
        self._fallback = fallback_child
        self._settle_msgs = max(1, int(settle_msgs))
        self._lock = threading.Lock()
        self._state = _State()

    def on_rewire(self) -> None:
        """Invalidate everything — the next controller may control a different
        point, and nothing observed under the previous one is evidence about it.
        """
        with self._lock:
            self._state = _State()

    def observe(self, child_frames: Iterable[str]) -> None:
        """Fold one TFMessage's child frame ids into the selection."""
        frames = set(child_frames)
        with self._lock:
            state = self._state
            if state.selection is not None:
                # Latched. A frame missing from a later message is not evidence
                # of anything — the control point is allowed to drop out for a
                # tick, and the readout must not follow it.
                return
            if VIRTUAL_TCP_FRAME in frames:
                self._state = replace(state, selection=VIRTUAL_TCP_FRAME)
                return
            if self._fallback in frames:
                seen = state.fallback_seen + 1
                self._state = replace(
                    state,
                    fallback_seen=seen,
                    selection=self._fallback if seen >= self._settle_msgs else None,
                )
            # A message carrying neither frame says nothing about the task
            # frame, so it neither advances nor resets the window.

    def selection(self) -> str | None:
        """The latched child frame, or None while the frame is still unsettled.

        None is the GUI's cue to withhold the task readout and disable task
        target entry — it means "the control frame is not known yet", which is
        exactly when a pose read or a goal written would be in the wrong frame.
        """
        return self._state.selection

    def seeded(self) -> bool:
        """Have the task target entries been seeded since the last rewire?"""
        return self._state.seeded

    def mark_seeded(self) -> None:
        with self._lock:
            self._state = replace(self._state, seeded=True)
