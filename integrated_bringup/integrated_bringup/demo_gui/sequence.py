"""Posture sequence of demo_controller_gui's Sequence tab — Tk- and rclpy-free.

A sequence is a list of steps; each names a hand preset and carries its own
move time and dwell. Playing it sends the presets' HAND targets one after
another to the active controller — the arm is left alone, and a preset's
``controller`` / ``robot_target`` are not used.

What lives here, so pytest can pin it:

- the step list and its file (``demo_gui_sequences_<scope>.json``),
- the check that turns the list into targets before anything is sent
  (``resolve_steps``) — a run plays that snapshot, not the live presets,
- the run itself (``SequenceRunner``): when the next step may go out, and
  every reason a run ends.

The controllers report no "trajectory finished", so a step is over when its
time has passed AND the measured hand is within a tolerance of the target. A
target the hand cannot reach (a joint limit the controller clamps to, a
mechanical end stop) therefore ends the run with a timeout that names the
joint, instead of the next step being sent on top of an unfinished move.
"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace

from .preset_toggle import atomic_write_json

TIME_RESOLUTION_S = 0.01
MIN_TIME_S = 0.01  # preset_speed.MIN_GRASP_TIME: below it a time is no speed request
MAX_TIME_S = 600.0
DEFAULT_TOLERANCE_DEG = 2.0
MIN_TOLERANCE_DEG = 0.1
MAX_TOLERANCE_DEG = 30.0

MODE_ONCE = "once"
MODE_LOOP = "loop"

# A measurement older than this is not evidence that the hand arrived.
STALE_AFTER_S = 0.5
# A step is given up after TIMEOUT_FACTOR x its expected duration + TIMEOUT_MARGIN_S.
TIMEOUT_FACTOR = 2.0
TIMEOUT_MARGIN_S = 1.0
# The speed set before a goal answers within the GUI's own 1 s timeout; a send
# that has not completed after this is a lost callback, not a slow service.
SEND_TIMEOUT_S = 3.0

IDLE = "idle"
READY = "ready"  # the current step is due to be sent
SENDING = "sending"  # handed to the GUI; its completion has not come back
MOVING = "moving"
DWELLING = "dwelling"


def round_time(value: float) -> float:
    """``value`` [s] on the 0.01 s grid."""
    return round(round(float(value) / TIME_RESOLUTION_S) * TIME_RESOLUTION_S, 2)


def clamp_time(value: float) -> float:
    """A move time: on the grid, within [MIN_TIME_S, MAX_TIME_S]."""
    return min(max(round_time(value), MIN_TIME_S), MAX_TIME_S)


def clamp_dwell(value: float) -> float:
    """A dwell: on the grid, within [0, MAX_TIME_S]."""
    return min(max(round_time(value), 0.0), MAX_TIME_S)


def clamp_tolerance(value: float) -> float:
    return min(max(round(float(value), 1), MIN_TOLERANCE_DEG), MAX_TOLERANCE_DEG)


@dataclass(frozen=True)
class SequenceStep:
    """One row of the list: a preset by name, how long the move to it takes
    and how long the hand rests there before the next step."""

    preset: str
    time_s: float
    dwell_s: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "time_s", clamp_time(self.time_s))
        object.__setattr__(self, "dwell_s", clamp_dwell(self.dwell_s))


@dataclass(frozen=True)
class SequenceFile:
    steps: tuple[SequenceStep, ...] = ()
    tolerance_deg: float = DEFAULT_TOLERANCE_DEG


def with_all_times(
    steps: Sequence[SequenceStep], time_s: float | None = None, dwell_s: float | None = None
) -> tuple[SequenceStep, ...]:
    """Every step with its move time and/or dwell replaced; None keeps a field."""
    changes = {}
    if time_s is not None:
        changes["time_s"] = time_s
    if dwell_s is not None:
        changes["dwell_s"] = dwell_s
    return tuple(replace(step, **changes) for step in steps)


def moved(steps: Sequence[SequenceStep], index: int, offset: int) -> tuple[tuple, int]:
    """``steps`` with row ``index`` moved by ``offset``, and where it ended up
    (unchanged at either end of the list)."""
    target = index + offset
    rows = list(steps)
    if not (0 <= index < len(rows)) or not (0 <= target < len(rows)):
        return tuple(rows), index
    rows[index], rows[target] = rows[target], rows[index]
    return tuple(rows), target


def _finite(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value) if math.isfinite(value) else None


def normalize_sequence_file(data: object) -> SequenceFile:
    """Read persisted data; a malformed step is dropped, never guessed at."""
    if not isinstance(data, dict):
        return SequenceFile()
    steps = []
    raw_steps = data.get("steps")
    for raw in raw_steps if isinstance(raw_steps, list) else []:
        if not isinstance(raw, dict):
            continue
        preset, time_s = raw.get("preset"), _finite(raw.get("time_s"))
        dwell_s = _finite(raw.get("dwell_s", 0.0))
        if not isinstance(preset, str) or not preset or time_s is None or dwell_s is None:
            continue
        steps.append(SequenceStep(preset, time_s, dwell_s))
    tolerance = _finite(data.get("tolerance_deg"))
    return SequenceFile(
        tuple(steps),
        DEFAULT_TOLERANCE_DEG if tolerance is None else clamp_tolerance(tolerance),
    )


def load_sequence_file(path: str) -> SequenceFile:
    try:
        with open(path) as sequence_file:
            return normalize_sequence_file(json.load(sequence_file))
    except (OSError, json.JSONDecodeError):
        return SequenceFile()


def save_sequence_file(path: str, sequence: SequenceFile) -> None:
    """Atomically persist the list beside the preset store."""
    atomic_write_json(
        path,
        {
            "steps": [
                {"preset": s.preset, "time_s": s.time_s, "dwell_s": s.dwell_s}
                for s in sequence.steps
            ],
            "tolerance_deg": clamp_tolerance(sequence.tolerance_deg),
        },
        prefix=".demo_gui_sequences_",
    )


def step_problem(step: SequenceStep, presets: Mapping[str, Mapping], hand_dof: int) -> str:
    """Why ``step`` cannot be played against ``presets`` ("" = it can)."""
    data = presets.get(step.preset)
    if data is None:
        return "preset no longer exists"
    positions = data.get("positions_deg", [])
    if not isinstance(positions, (list, tuple)) or len(positions) != hand_dof:
        return f"preset does not match the hand DoF ({hand_dof})"
    if any(_finite(v) is None for v in positions):
        return "preset holds a non-numeric position"
    return ""


@dataclass(frozen=True)
class ResolvedStep:
    """A step with its target fixed [rad] — what a run plays."""

    preset: str
    target_rad: tuple[float, ...]
    time_s: float
    dwell_s: float


def resolve_steps(
    steps: Sequence[SequenceStep], presets: Mapping[str, Mapping], hand_dof: int
) -> tuple[tuple[ResolvedStep, ...], str]:
    """(targets of every step, "") — or ((), why the list cannot be played).

    All or nothing: a list with one unusable row is refused before the first
    goal, not abandoned halfway through.
    """
    if not steps:
        return (), "the sequence is empty"
    resolved = []
    for number, step in enumerate(steps, start=1):
        if problem := step_problem(step, presets, hand_dof):
            return (), f"step {number} '{step.preset}': {problem}"
        target = tuple(math.radians(float(v)) for v in presets[step.preset]["positions_deg"])
        resolved.append(ResolvedStep(step.preset, target, step.time_s, step.dwell_s))
    return tuple(resolved), ""


class SequenceRunner:
    """Plays resolved steps. The GUI owns the clock and the publishing:

    - ``tick`` is called periodically; it returns a step when one is due to be
      sent (``token`` then identifies that send),
    - the GUI sets the hand speed, publishes, and reports back with ``on_sent``
      — the step's time is counted from there, because the speed set before a
      goal is a service round trip,
    - ``stop`` ends the run. Nothing is sent after it: a send still in flight is
      refused by ``awaiting``. A trajectory already started runs to its end.
    """

    def __init__(self) -> None:
        self.state = IDLE
        self.mode = MODE_ONCE
        self.index = 0
        self.cycle = 0  # completed passes through the list
        self.token = 0
        self.result = ""  # how the last run ended
        self.aborted = False  # the last run ended on its own, short of its end
        self._steps: tuple[ResolvedStep, ...] = ()
        self._names: tuple[str, ...] = ()
        self._tolerance_rad = math.radians(DEFAULT_TOLERANCE_DEG)
        self._since = 0.0  # when the current state was entered
        self._expected_s = 0.0

    # ── Control ──────────────────────────────────────────────────────────────

    @property
    def running(self) -> bool:
        return self.state != IDLE

    def start(
        self,
        steps: Sequence[ResolvedStep],
        mode: str,
        tolerance_deg: float = DEFAULT_TOLERANCE_DEG,
        joint_names: Sequence[str] = (),
    ) -> str:
        """Begin a run; returns why not ("" = started)."""
        if self.running:
            return "a sequence is already playing"
        if not steps:
            return "the sequence is empty"
        if mode not in (MODE_ONCE, MODE_LOOP):
            return f"unknown mode '{mode}'"
        self._steps = tuple(steps)
        self._names = tuple(joint_names)
        self._tolerance_rad = math.radians(clamp_tolerance(tolerance_deg))
        self.mode, self.index, self.cycle = mode, 0, 0
        self.result, self.aborted = "", False
        self.state = READY
        return ""

    def stop(self, reason: str = "stopped by the operator") -> None:
        if self.running:
            self._end(f"{reason} at {self._where()}", aborted=False)

    def awaiting(self, token: int) -> bool:
        """Is the send identified by ``token`` still wanted?"""
        return self.state == SENDING and token == self.token

    def on_sent(
        self, token: int, now: float, refusal: str = "", min_duration: float | None = None
    ) -> None:
        """The GUI finished the send ``token``: published (``refusal`` empty) or
        withheld. ``min_duration`` [s] is the speed plan's lower bound on the
        move, None when no speed was set."""
        if not self.awaiting(token):
            return
        if refusal:
            self._abort(f"{self._where()} was not sent — {refusal}")
            return
        step = self._steps[self.index]
        self._expected_s = max(step.time_s, min_duration or 0.0)
        self._enter(MOVING, now)

    def tick(
        self,
        now: float,
        measured: Sequence[float],
        measured_age_s: float | None,
        block: str = "",
    ) -> ResolvedStep | None:
        """Advance; returns the step to send now, if one is due.

        ``measured`` [rad] is the latest hand position and ``measured_age_s``
        how old it is (None = never received). ``block`` is a reason the run
        must not continue (E-STOP, another controller, a mode that refuses
        goals) — it ends the run whatever the state.
        """
        if not self.running:
            return None
        if block:
            self._abort(f"{block} ({self._where()})")
            return None
        if self.state == SENDING:
            if now - self._since > SEND_TIMEOUT_S:
                self._abort(f"the send of {self._where()} never completed")
            return None
        if self.state == MOVING and not self._moving_done(now, measured, measured_age_s):
            return None
        if self.state == DWELLING and now - self._since < self._steps[self.index].dwell_s:
            return None
        if self.state in (MOVING, DWELLING) and not self._advance():
            return None
        # READY: hand the step out.
        self.token += 1
        self._enter(SENDING, now)
        return self._steps[self.index]

    # ── Readout ──────────────────────────────────────────────────────────────

    @property
    def expected_s(self) -> float:
        """How long the current move takes at least (MOVING only)."""
        return self._expected_s

    def status(self, now: float) -> str:
        if not self.running:
            return self.result or "idle"
        head = f"playing ({self.mode}) — cycle {self.cycle + 1}, {self._where()}"
        elapsed = max(now - self._since, 0.0)
        if self.state == MOVING:
            return f"{head}: moving {elapsed:.1f} / {self._expected_s:.2f} s"
        if self.state == DWELLING:
            return f"{head}: dwell {elapsed:.1f} / {self._steps[self.index].dwell_s:.2f} s"
        return f"{head}: sending"

    # ── Internals ────────────────────────────────────────────────────────────

    def _where(self) -> str:
        step = self._steps[self.index]
        return f"step {self.index + 1}/{len(self._steps)} '{step.preset}'"

    def _enter(self, state: str, now: float) -> None:
        self.state, self._since = state, now

    def _end(self, result: str, *, aborted: bool) -> None:
        self.state, self.result, self.aborted = IDLE, result, aborted

    def _abort(self, reason: str) -> None:
        self._end(f"aborted — {reason}", aborted=True)

    def _moving_done(self, now: float, measured: Sequence[float], age: float | None) -> bool:
        """Is the move over? Enters DWELLING when it is and a dwell is set."""
        elapsed = now - self._since
        step = self._steps[self.index]
        fresh = age is not None and age <= STALE_AFTER_S
        errors = [abs(t - m) for t, m in zip(step.target_rad, measured, strict=False)]
        comparable = len(measured) == len(step.target_rad)
        reached = fresh and comparable and max(errors, default=0.0) <= self._tolerance_rad
        if elapsed >= self._expected_s and reached:
            if step.dwell_s > 0.0:
                self._enter(DWELLING, now)
                return False
            return True
        timeout = TIMEOUT_FACTOR * self._expected_s + TIMEOUT_MARGIN_S
        if elapsed > timeout:
            self._abort(
                f"{self._where()} not reached within {timeout:.2f} s: {self._why(age, errors, comparable)}"
            )
        return False

    def _why(self, age: float | None, errors: list[float], comparable: bool) -> str:
        if age is None:
            return "no hand joint state was ever received"
        if age > STALE_AFTER_S:
            return f"the hand joint state is stale (last one {age:.1f} s ago)"
        if not comparable or not errors:
            return "the hand joint state does not match the target's size"
        worst = max(range(len(errors)), key=errors.__getitem__)
        name = self._names[worst] if worst < len(self._names) else f"joint {worst + 1}"
        return (
            f"{name} is {math.degrees(errors[worst]):.1f} deg off "
            f"(tolerance {math.degrees(self._tolerance_rad):.1f} deg)"
        )

    def _advance(self) -> bool:
        """Move to the next step; False when that ended the run."""
        self.index += 1
        if self.index < len(self._steps):
            self.state = READY
            return True
        self.cycle += 1
        if self.mode == MODE_ONCE:
            self.index = len(self._steps) - 1
            self._end(f"done — {len(self._steps)} steps played once", aborted=False)
            return False
        self.index = 0
        self.state = READY
        return True
