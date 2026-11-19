"""Qt-free, rclpy-free model of the teleop input: config, sources, button rules.

Everything here runs on plain floats for time (``time.monotonic()`` in the
node, a fake clock in the tests).

- ``VirtualPad`` turns keyboard and mouse events into a pad state. It is
  touched only by the GUI thread (events and the publish tick both run there).
- ``PadRelay`` holds the latest Joy of a real pad. Its writer is the rclpy
  executor thread and its reader the publish tick, so it locks.
- ``ButtonActions`` decides the service calls from the frames: one E-STOP per
  press of the E-STOP button, one reset per continuous hold of the reset
  button while the deadman stays released.

A ``PadFrame`` is what one publish tick sends; the GUI draws the same frame,
so the window never shows a state that was not published.

Safety rules (each one has a test):

- Key auto-repeat is ignored (X11 delivers a held key as release/press
  pairs; obeying them would make the deadman flicker).
- Focus loss releases everything; a key still held when focus comes back
  stays released until it is pressed again.
- A button press is latched until the next tick reads it, so a press and
  release inside one tick still counts once.
- In ``pad`` mode, a silent upstream (``pad_timeout_s``) or a Joy of the wrong
  size publishes the released state, never the last value.
"""

from __future__ import annotations

import math
import threading
from collections.abc import Sequence
from dataclasses import dataclass, field

from rtc_teleop_input.layout import GamepadLayout, LayoutError

INPUTS = ("virtual", "pad")
SKINS = ("f710", "xbox")
MAX_RATE_HZ = 1000.0
STICK_SIDES = ("left", "right")
# Mouse drags: screen right/up -> Joy axes. game_controller_node reads a stick
# pushed LEFT or UP as positive, so x flips and y does not.
_STICK_AXES = {"left": ("leftx", "lefty"), "right": ("rightx", "righty")}
# A trigger rests at 0 and reads -1 fully pressed (same source convention).
TRIGGER_PRESSED = -1.0


class ConfigError(ValueError):
    """A parameter is missing or out of range; the node refuses to start."""


@dataclass(frozen=True)
class TeleopConfig:
    """Node parameters. Topic and controller names have no default (robot-agnostic)."""

    input: str = "virtual"
    output_topic: str = ""
    rate_hz: float = 50.0
    skin: str = "f710"
    # Key deflection: a held key pushes its axis this far (1.0 = full stick).
    deflection: float = 0.25
    # "<axis><+|->:<Qt key name>" and "<button>:<Qt key name>" (Qt::Key_<name>).
    key_axes: tuple[str, ...] = (
        "lefty+:W",
        "lefty-:S",
        "leftx+:A",
        "leftx-:D",
        "rightx+:Left",
        "rightx-:Right",
    )
    key_buttons: tuple[str, ...] = ("leftshoulder:Space", "b:Escape", "back:Backspace")
    pad_topic: str = "joy"
    pad_timeout_s: float = 0.25
    estop_button: str = "b"
    reset_button: str = "back"
    deadman_button: str = "leftshoulder"
    reset_hold_s: float = 1.0
    estop_service: str = "/rtc_cm/trigger_estop"
    reset_service: str = "/rtc_cm/reset_fault"
    # Empty: the reset button does nothing (reset_fault refuses an empty name).
    reset_controller_name: str = ""
    status_topics: tuple[str, ...] = ()

    @property
    def reset_enabled(self) -> bool:
        return bool(self.reset_controller_name)


@dataclass(frozen=True)
class KeyBindings:
    # key name -> (axis index, +1 / -1)
    axes: dict[str, tuple[int, int]]
    # key name -> button index
    buttons: dict[str, int]

    def keys(self) -> tuple[str, ...]:
        return tuple(self.axes) + tuple(self.buttons)


def _finite_in(value: float, lo: float, hi: float, lo_open: bool = True) -> bool:
    return math.isfinite(value) and (value > lo if lo_open else value >= lo) and value <= hi


def parse_bindings(config: TeleopConfig, layout: GamepadLayout) -> KeyBindings:
    axes: dict[str, tuple[int, int]] = {}
    buttons: dict[str, int] = {}
    seen_targets: set[tuple[str, int]] = set()
    try:
        for entry in config.key_axes:
            target, sep, key = entry.partition(":")
            if not sep or not key or len(target) < 2 or target[-1] not in "+-":
                raise ConfigError(f"key_axes entry '{entry}' is not '<axis><+|->:<key>'")
            index = layout.axis_index(target[:-1])
            sign = 1 if target[-1] == "+" else -1
            if (target, 0) in seen_targets:
                raise ConfigError(f"key_axes binds '{target}' twice")
            seen_targets.add((target, 0))
            if key in axes or key in buttons:
                raise ConfigError(f"key '{key}' is bound twice")
            axes[key] = (index, sign)
        for entry in config.key_buttons:
            target, sep, key = entry.partition(":")
            if not sep or not key or not target:
                raise ConfigError(f"key_buttons entry '{entry}' is not '<button>:<key>'")
            index = layout.button_index(target)
            if (target, 1) in seen_targets:
                raise ConfigError(f"key_buttons binds '{target}' twice")
            seen_targets.add((target, 1))
            if key in axes or key in buttons:
                raise ConfigError(f"key '{key}' is bound twice")
            buttons[key] = index
    except LayoutError as exc:
        raise ConfigError(str(exc)) from exc
    return KeyBindings(axes=axes, buttons=buttons)


def validate_config(config: TeleopConfig, layout: GamepadLayout) -> KeyBindings:
    """Checks every parameter; returns the parsed key bindings."""
    if config.input not in INPUTS:
        raise ConfigError(f"input must be one of {INPUTS}, got '{config.input}'")
    if not config.output_topic:
        raise ConfigError("output_topic is required (the Joy topic the controller reads)")
    if not _finite_in(config.rate_hz, 0.0, MAX_RATE_HZ):
        raise ConfigError(f"rate_hz must be in (0, {MAX_RATE_HZ:g}], got {config.rate_hz}")
    if config.skin not in SKINS:
        raise ConfigError(f"skin must be one of {SKINS}, got '{config.skin}'")
    if not _finite_in(config.deflection, 0.0, 1.0):
        raise ConfigError(f"deflection must be in (0, 1], got {config.deflection}")
    if not _finite_in(config.pad_timeout_s, 0.0, 10.0):
        raise ConfigError(f"pad_timeout_s must be in (0, 10], got {config.pad_timeout_s}")
    if not _finite_in(config.reset_hold_s, 0.0, 10.0):
        raise ConfigError(f"reset_hold_s must be in (0, 10], got {config.reset_hold_s}")
    if config.input == "pad" and not config.pad_topic:
        raise ConfigError("pad_topic is required for input 'pad'")
    if not config.estop_service or not config.reset_service:
        raise ConfigError("estop_service and reset_service must not be empty")
    roles = (config.estop_button, config.reset_button, config.deadman_button)
    try:
        for name in roles:
            layout.button_index(name)
    except LayoutError as exc:
        raise ConfigError(str(exc)) from exc
    if len(set(roles)) != len(roles):
        raise ConfigError("estop_button, reset_button and deadman_button must differ")
    if len(set(config.status_topics)) != len(config.status_topics) or "" in config.status_topics:
        raise ConfigError("status_topics must be distinct, non-empty topic names")
    return parse_bindings(config, layout)


@dataclass(frozen=True)
class PadFrame:
    """One publish tick: what goes out as Joy and what the GUI draws."""

    source: str
    axes: tuple[float, ...]
    buttons: tuple[int, ...]
    # Real presses of each button since the previous frame (auto-repeat excluded).
    presses: tuple[int, ...]
    # pad: the upstream has been silent longer than pad_timeout_s (or never spoke).
    lost: bool = False


def released_frame(
    layout: GamepadLayout, source: str, presses: tuple[int, ...] | None = None, lost: bool = False
) -> PadFrame:
    zeros = (0,) * len(layout.buttons)
    return PadFrame(
        source=source,
        axes=(0.0,) * len(layout.axes),
        buttons=zeros,
        presses=presses if presses is not None else zeros,
        lost=lost,
    )


def _clamp_unit(value: float) -> float:
    return max(-1.0, min(1.0, value))


class VirtualPad:
    """Keyboard and mouse -> pad state. GUI thread only."""

    def __init__(self, layout: GamepadLayout, bindings: KeyBindings, deflection: float) -> None:
        self._layout = layout
        self._bindings = bindings
        self._deflection = deflection
        self._keys_down: set[str] = set()
        self._mouse_sticks: dict[str, tuple[float, float]] = {}
        self._mouse_buttons: set[int] = set()
        self._mouse_triggers: set[int] = set()
        self._presses = [0] * len(layout.buttons)
        self._tapped: set[int] = set()

    # ── events ──────────────────────────────────────────────────────────

    def handles_key(self, key: str) -> bool:
        return key in self._bindings.axes or key in self._bindings.buttons

    def key_event(self, key: str, pressed: bool, auto_repeat: bool) -> bool:
        """Applies one key event; returns whether the key is bound."""
        if not self.handles_key(key):
            return False
        if auto_repeat:
            return True
        if pressed == (key in self._keys_down):
            # a press of a held key, or a release of a key released by focus loss
            return True
        before = self._held_buttons()
        if pressed:
            self._keys_down.add(key)
        else:
            self._keys_down.discard(key)
        self._note_presses(before)
        return True

    def drag_stick(self, side: str, right: float, up: float) -> None:
        if side not in STICK_SIDES:
            raise ValueError(f"stick side must be one of {STICK_SIDES}, got '{side}'")
        self._mouse_sticks[side] = (right, up)

    def release_stick(self, side: str) -> None:
        self._mouse_sticks.pop(side, None)

    def mouse_control(self, name: str, pressed: bool) -> None:
        """A drawn button (layout button name) or trigger (axis name) under the mouse."""
        if name in self._layout.axes:
            index = self._layout.axis_index(name)
            if pressed:
                self._mouse_triggers.add(index)
            else:
                self._mouse_triggers.discard(index)
            return
        index = self._layout.button_index(name)
        before = self._held_buttons()
        if pressed:
            self._mouse_buttons.add(index)
        else:
            self._mouse_buttons.discard(index)
        self._note_presses(before)

    def focus_lost(self) -> None:
        """Releases every key, stick and button. Pending presses stay latched:
        an E-STOP pressed just before the window lost focus still fires."""
        self._keys_down.clear()
        self._mouse_sticks.clear()
        self._mouse_buttons.clear()
        self._mouse_triggers.clear()
        self._tapped.clear()

    # ── tick ────────────────────────────────────────────────────────────

    def frame(self) -> PadFrame:
        """The state to publish now; consumes the latched presses."""
        axes = [0.0] * len(self._layout.axes)
        for key in self._keys_down:
            binding = self._bindings.axes.get(key)
            if binding is not None:
                index, sign = binding
                axes[index] += sign * self._deflection
        for side, (right, up) in self._mouse_sticks.items():
            x_name, y_name = _STICK_AXES[side]
            axes[self._layout.axis_index(x_name)] += -right
            axes[self._layout.axis_index(y_name)] += up
        for index in self._mouse_triggers:
            axes[index] += TRIGGER_PRESSED
        held = self._held_buttons() | self._tapped
        buttons = tuple(1 if i in held else 0 for i in range(len(self._layout.buttons)))
        presses = tuple(self._presses)
        self._presses = [0] * len(self._layout.buttons)
        self._tapped.clear()
        return PadFrame(
            source="virtual",
            axes=tuple(_clamp_unit(v) for v in axes),
            buttons=buttons,
            presses=presses,
        )

    # ── internals ───────────────────────────────────────────────────────

    def _held_buttons(self) -> set[int]:
        held = set(self._mouse_buttons)
        for key in self._keys_down:
            index = self._bindings.buttons.get(key)
            if index is not None:
                held.add(index)
        return held

    def _note_presses(self, before: set[int]) -> None:
        for index in self._held_buttons() - before:
            self._presses[index] += 1
            self._tapped.add(index)


class PadRelay:
    """Latest Joy of a real pad. Written by the executor thread, read by the tick."""

    def __init__(self, layout: GamepadLayout, timeout_s: float) -> None:
        self._layout = layout
        self._timeout_s = timeout_s
        self._lock = threading.Lock()
        self._axes: tuple[float, ...] | None = None
        self._buttons: tuple[int, ...] | None = None
        self._prev_buttons = (0,) * len(layout.buttons)
        self._received_s: float | None = None
        self._presses = [0] * len(layout.buttons)
        self._tapped: set[int] = set()
        self._rejects = 0

    def on_upstream(self, axes: Sequence[float], buttons: Sequence[int], now: float) -> None:
        """One upstream Joy; ``now`` is the monotonic receive time (not the stamp)."""
        with self._lock:
            self._received_s = now
            if len(axes) != len(self._layout.axes) or len(buttons) != len(self._layout.buttons):
                self._rejects += 1
                self._axes = None
                self._buttons = None
                return
            current = tuple(int(b) for b in buttons)
            for i, (was, now_b) in enumerate(zip(self._prev_buttons, current, strict=True)):
                if now_b == 1 and was != 1:
                    self._presses[i] += 1
                    self._tapped.add(i)
            self._prev_buttons = current
            self._axes = tuple(axes)
            self._buttons = current

    @property
    def rejects(self) -> int:
        with self._lock:
            return self._rejects

    def frame(self, now: float) -> PadFrame:
        with self._lock:
            presses = tuple(self._presses)
            self._presses = [0] * len(self._layout.buttons)
            tapped = self._tapped
            self._tapped = set()
            silent = self._received_s is None or now - self._received_s > self._timeout_s
            if silent:
                # The pad comes back from a released state: a button still held
                # then is a new press.
                self._prev_buttons = (0,) * len(self._layout.buttons)
                self._axes = None
                self._buttons = None
                return released_frame(self._layout, "pad", presses, lost=True)
            if self._axes is None or self._buttons is None:
                return released_frame(self._layout, "pad", presses)
            buttons = tuple(1 if i in tapped else b for i, b in enumerate(self._buttons))
            return PadFrame(source="pad", axes=self._axes, buttons=buttons, presses=presses)


@dataclass(frozen=True)
class Actions:
    estop_calls: int = 0
    reset_call: bool = False


class ButtonActions:
    """Service calls from the frames. Runs on the tick (GUI) thread."""

    def __init__(self, layout: GamepadLayout, config: TeleopConfig) -> None:
        self._estop = layout.button_index(config.estop_button)
        self._reset = layout.button_index(config.reset_button)
        self._deadman = layout.button_index(config.deadman_button)
        self._hold_s = config.reset_hold_s
        self._reset_enabled = config.reset_enabled
        self._gui_estops = 0
        self._hold_start: float | None = None
        self._fired = False

    def request_estop(self) -> None:
        """The window's E-STOP button: one call at the next tick."""
        self._gui_estops += 1

    def reset_progress(self, now: float) -> float | None:
        """Fraction of the reset hold done, None when no hold is running."""
        if self._hold_start is None or self._fired or not self._reset_enabled:
            return None
        return min(1.0, (now - self._hold_start) / self._hold_s)

    def step(self, frame: PadFrame, now: float) -> Actions:
        estops = frame.presses[self._estop] + self._gui_estops
        self._gui_estops = 0
        return Actions(estop_calls=estops, reset_call=self._step_reset(frame, now))

    def _step_reset(self, frame: PadFrame, now: float) -> bool:
        if not frame.buttons[self._reset]:
            self._hold_start = None
            self._fired = False
            return False
        if frame.buttons[self._deadman]:
            # the deadman must stay released for the whole hold: start again
            # (a tap inside one tick is latched into buttons, so it shows here)
            self._hold_start = None
            return False
        if frame.presses[self._reset]:
            # released and pressed again since the last tick: a new hold
            self._hold_start = now
            self._fired = False
            return False
        if self._hold_start is None:
            self._hold_start = now
            return False
        if self._fired or now - self._hold_start < self._hold_s:
            return False
        self._fired = True
        return self._reset_enabled


LEVEL_NAMES = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}


@dataclass(frozen=True)
class StatusEntry:
    topic: str
    name: str
    level: int
    message: str
    values: tuple[tuple[str, str], ...]
    received_s: float

    @property
    def level_name(self) -> str:
        return LEVEL_NAMES.get(self.level, f"level {self.level}")


@dataclass
class StatusBoard:
    """Latest DiagnosticStatus per topic, in the configured topic order. GUI thread."""

    topics: tuple[str, ...]
    _entries: dict[str, StatusEntry] = field(default_factory=dict)

    def update(self, entry: StatusEntry) -> None:
        if entry.topic in self.topics:
            self._entries[entry.topic] = entry

    def rows(self) -> list[StatusEntry]:
        return [self._entries[t] for t in self.topics if t in self._entries]
