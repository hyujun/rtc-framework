"""Qt-free, rclpy-free model of the teleop input: config, sources, button rules.

Everything here runs on plain floats for time (``time.monotonic()`` in the
node, a fake clock in the tests).

- ``VirtualPad`` turns keyboard and mouse events into a pad state. It is
  touched only by the GUI thread (events and the publish tick both run there).
- ``PadRelay`` holds the latest Joy of a real pad. Its writer is the rclpy
  executor thread and its reader the publish tick, so it locks.
- ``ButtonActions`` decides the service calls from the frames: one E-STOP per
  press of the E-STOP button, one call per continuous hold of a hold button
  (reset, switch, clear E-STOP, trigger) while the deadman stays released.
- ``HelpCombo`` and ``help_entries`` drive the help view: which control does
  what, toggled by a button combination of the real pad (or a key, in the
  window).

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
- A hold starts again (sends nothing) while the deadman is held and while
  two or more hold buttons are held.
- An E-STOP press (pad, key or window) cancels every hold running then: that
  button sends nothing until it is released and pressed again. A clear held
  through an E-STOP must not lower that E-STOP, not even after the E-STOP
  button is let go. The clear hold also needs the E-STOP button released.
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
# An axis in help_combo counts as pressed at or below this (half a trigger).
HELP_AXIS_PRESSED = -0.5
# Buttons of the layout the window draws; the others (misc1, paddles,
# touchpad) cannot be seen or clicked, so no role may use them.
DRAWN_BUTTONS = (
    "a",
    "b",
    "x",
    "y",
    "back",
    "guide",
    "start",
    "leftstick",
    "rightstick",
    "leftshoulder",
    "rightshoulder",
    "dpad_up",
    "dpad_down",
    "dpad_left",
    "dpad_right",
)
HOLD_KINDS = ("reset", "switch", "clear", "trigger")


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
    # The other hold buttons (one call per hold of hold_s, deadman released).
    # Empty lists / names: no such button.
    hold_s: float = 1.0
    # "<button>:<controller>" -> switch_controller activating that controller.
    switch_buttons: tuple[str, ...] = ()
    switch_service: str = "/rtc_cm/switch_controller"
    # Lowers the global E-STOP only when its latched reason is
    # clear_estop_reason (the service makes the caller echo it).
    clear_estop_button: str = ""
    clear_estop_service: str = "/rtc_cm/clear_estop"
    clear_estop_reason: str = ""
    # "<button>:<service>" -> one std_srvs/Trigger call.
    trigger_buttons: tuple[str, ...] = ()
    # Latched state shown in the window; empty: not shown.
    active_controller_topic: str = "/rtc_cm/active_controller_name"
    estop_status_topic: str = "/system/estop_status"
    # Help: a key of the window, and a pad combination (layout axis or button
    # names, all pressed at once; an axis counts at <= HELP_AXIS_PRESSED).
    help_key: str = "F1"
    help_combo: tuple[str, ...] = ()
    # "<axis or button>:<text>": what a control does for the consumer.
    help_labels: tuple[str, ...] = ()

    @property
    def reset_enabled(self) -> bool:
        return bool(self.reset_controller_name)


@dataclass(frozen=True)
class HoldEntry:
    """One hold button and the call it sends."""

    kind: str  # one of HOLD_KINDS
    button: str
    # reset / switch: the controller; trigger: the service; clear: ""
    target: str
    enabled: bool = True

    @property
    def key(self) -> str:
        """Names the call in replies and counters ("reset", "clear", "switch:<c>", ...)."""
        return f"{self.kind}:{self.target}" if self.kind in ("switch", "trigger") else self.kind


def _split_targets(entries: Sequence[str], param: str, what: str) -> list[tuple[str, str]]:
    pairs = []
    for entry in entries:
        button, sep, target = entry.partition(":")
        if not sep or not button or not target:
            raise ConfigError(f"{param} entry '{entry}' is not '<button>:<{what}>'")
        pairs.append((button, target))
    return pairs


def hold_entries(config: TeleopConfig) -> tuple[HoldEntry, ...]:
    """Every hold button in a fixed order: reset, switches, clear, triggers."""
    holds = [
        HoldEntry("reset", config.reset_button, config.reset_controller_name, config.reset_enabled)
    ]
    for button, target in _split_targets(config.switch_buttons, "switch_buttons", "controller"):
        holds.append(HoldEntry("switch", button, target))
    if config.clear_estop_button:
        holds.append(HoldEntry("clear", config.clear_estop_button, ""))
    for button, target in _split_targets(config.trigger_buttons, "trigger_buttons", "service"):
        holds.append(HoldEntry("trigger", button, target))
    return tuple(holds)


@dataclass(frozen=True)
class KeyBindings:
    # key name -> (axis index, +1 / -1)
    axes: dict[str, tuple[int, int]]
    # key name -> button index
    buttons: dict[str, int]

    def keys(self) -> tuple[str, ...]:
        return tuple(self.axes) + tuple(self.buttons)


def _finite_in(value: float, lo: float, hi: float) -> bool:
    """value in (lo, hi] and finite."""
    return math.isfinite(value) and lo < value <= hi


def parse_bindings(config: TeleopConfig, layout: GamepadLayout) -> KeyBindings:
    axes: dict[str, tuple[int, int]] = {}
    buttons: dict[str, int] = {}

    def split(entry: str, param: str, shape: str, seen: set[str]) -> tuple[str, str]:
        target, sep, key = entry.partition(":")
        if not sep or not key or not target:
            raise ConfigError(f"{param} entry '{entry}' is not '{shape}'")
        if target in seen:
            raise ConfigError(f"{param} binds '{target}' twice")
        seen.add(target)
        if key in axes or key in buttons:
            raise ConfigError(f"key '{key}' is bound twice")
        return target, key

    try:
        seen: set[str] = set()
        for entry in config.key_axes:
            target, key = split(entry, "key_axes", "<axis><+|->:<key>", seen)
            if len(target) < 2 or target[-1] not in "+-":
                raise ConfigError(f"key_axes entry '{entry}' is not '<axis><+|->:<key>'")
            axes[key] = (layout.axis_index(target[:-1]), 1 if target[-1] == "+" else -1)
        seen = set()
        for entry in config.key_buttons:
            target, key = split(entry, "key_buttons", "<button>:<key>", seen)
            buttons[key] = layout.button_index(target)
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
    period_ms = 1000.0 / config.rate_hz
    if abs(period_ms - round(period_ms)) > 1e-9:
        # the Qt publish timer counts whole milliseconds: 300 Hz would run at 333 Hz
        raise ConfigError(
            "rate_hz must give a whole-millisecond period (50, 100, 250, ...; the Qt "
            f"publish timer counts milliseconds), got "
            f"{config.rate_hz:g} ({period_ms:.3f} ms)"
        )
    if config.skin not in SKINS:
        raise ConfigError(f"skin must be one of {SKINS}, got '{config.skin}'")
    if not _finite_in(config.deflection, 0.0, 1.0):
        raise ConfigError(f"deflection must be in (0, 1], got {config.deflection}")
    if not _finite_in(config.pad_timeout_s, 0.0, 10.0):
        raise ConfigError(f"pad_timeout_s must be in (0, 10], got {config.pad_timeout_s}")
    if not _finite_in(config.reset_hold_s, 0.0, 10.0):
        raise ConfigError(f"reset_hold_s must be in (0, 10], got {config.reset_hold_s}")
    if not _finite_in(config.hold_s, 0.0, 10.0):
        raise ConfigError(f"hold_s must be in (0, 10], got {config.hold_s}")
    if config.input == "pad" and not config.pad_topic:
        raise ConfigError("pad_topic is required for input 'pad'")
    if not config.estop_service or not config.reset_service:
        raise ConfigError("estop_service and reset_service must not be empty")
    holds = hold_entries(config)
    if config.switch_buttons and not config.switch_service:
        raise ConfigError("switch_service must not be empty when switch_buttons are set")
    if config.clear_estop_button and not (
        config.clear_estop_service and config.clear_estop_reason
    ):
        raise ConfigError(
            "clear_estop_button needs clear_estop_service and clear_estop_reason (the "
            "latched reason this button may clear)"
        )
    roles = (config.estop_button, config.deadman_button, *(h.button for h in holds))
    try:
        for name in roles:
            layout.button_index(name)
    except LayoutError as exc:
        raise ConfigError(str(exc)) from exc
    undrawn = sorted({name for name in roles if name not in DRAWN_BUTTONS})
    if undrawn:
        raise ConfigError(f"button roles must be drawn buttons, not {', '.join(undrawn)}")
    if len(set(roles)) != len(roles):
        raise ConfigError(
            "estop_button, deadman_button, reset_button and every switch, clear and "
            "trigger button must differ"
        )
    if len(set(config.status_topics)) != len(config.status_topics) or "" in config.status_topics:
        raise ConfigError("status_topics must be distinct, non-empty topic names")
    _check_help(config, layout, set(roles))
    bindings = parse_bindings(config, layout)
    if config.help_key in (*bindings.axes, *bindings.buttons):
        raise ConfigError(f"help_key '{config.help_key}' is also bound in key_axes / key_buttons")
    return bindings


def _check_help(config: TeleopConfig, layout: GamepadLayout, roles: set[str]) -> None:
    every = set(layout.axes) | set(layout.buttons)
    combo = config.help_combo
    if len(set(combo)) != len(combo):
        raise ConfigError("help_combo names a control twice")
    for name in combo:
        if name not in every:
            raise ConfigError(f"help_combo: unknown control '{name}'")
        if name in roles:
            raise ConfigError(f"help_combo: '{name}' already has a button role")
        if name in layout.buttons and name not in DRAWN_BUTTONS:
            raise ConfigError(f"help_combo: '{name}' is not a drawn button")
    seen = set()
    for entry in config.help_labels:
        name, sep, text = entry.partition(":")
        if not sep or not name or not text.strip():
            raise ConfigError(f"help_labels entry '{entry}' is not '<control>:<text>'")
        if name not in every:
            raise ConfigError(f"help_labels: unknown control '{name}'")
        if name in layout.buttons and name not in DRAWN_BUTTONS:
            raise ConfigError(f"help_labels: '{name}' is not a drawn button")
        if name in seen:
            raise ConfigError(f"help_labels labels '{name}' twice")
        seen.add(name)


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
            # a press since the last tick is published even if already released
            buttons = tuple(1 if n else b for n, b in zip(presses, self._buttons, strict=True))
            return PadFrame(source="pad", axes=self._axes, buttons=buttons, presses=presses)


@dataclass(frozen=True)
class Actions:
    estop_calls: int = 0
    # the holds completed this tick (enabled ones only), in hold_entries order
    fired: tuple[HoldEntry, ...] = ()

    @property
    def reset_call(self) -> bool:
        return any(h.kind == "reset" for h in self.fired)


class ButtonActions:
    """Service calls from the frames. Runs on the tick (GUI) thread.

    One hold state per hold button. Each one starts again while the deadman is
    held and while two or more hold buttons are held; the clear hold also while
    the E-STOP button is held. An E-STOP press spends every hold held then
    (sent nothing, and nothing more until its button is released). A hold
    sends once.
    """

    def __init__(self, layout: GamepadLayout, config: TeleopConfig) -> None:
        self._estop = layout.button_index(config.estop_button)
        self._deadman = layout.button_index(config.deadman_button)
        self.holds = hold_entries(config)
        self._index = tuple(layout.button_index(h.button) for h in self.holds)
        self._hold_s = tuple(
            config.reset_hold_s if h.kind == "reset" else config.hold_s for h in self.holds
        )
        self._gui_estops = 0
        self._start: list[float | None] = [None] * len(self.holds)
        self._fired = [False] * len(self.holds)

    def request_estop(self) -> None:
        """The window's E-STOP button: one call at the next tick."""
        self._gui_estops += 1

    def progress(self, entry: int, now: float) -> float | None:
        """Fraction of hold ``entry`` done, None when no hold is running."""
        start = self._start[entry]
        if start is None or self._fired[entry] or not self.holds[entry].enabled:
            return None
        return min(1.0, (now - start) / self._hold_s[entry])

    def reset_progress(self, now: float) -> float | None:
        """Fraction of the reset hold done, None when no hold is running."""
        return self.progress(0, now)  # hold_entries puts reset first

    def step(self, frame: PadFrame, now: float) -> Actions:
        estops = frame.presses[self._estop] + self._gui_estops
        self._gui_estops = 0
        held = sum(1 for i in self._index if frame.buttons[i])
        # the deadman must stay released for the whole hold (a tap inside one
        # tick is latched into buttons, so it shows here); an E-STOP press or
        # a second hold button also starts every hold again
        restart = bool(frame.buttons[self._deadman]) or held > 1
        if estops:
            # spent, not restarted: a hold kept through the E-STOP would
            # otherwise send hold_s after the E-STOP button is let go
            for entry, index in enumerate(self._index):
                if frame.buttons[index]:
                    self._start[entry] = None
                    self._fired[entry] = True
            restart = True
        fired = []
        for entry, hold in enumerate(self.holds):
            blocked = restart or (hold.kind == "clear" and bool(frame.buttons[self._estop]))
            if self._step_hold(entry, frame, now, blocked):
                fired.append(hold)
        return Actions(estop_calls=estops, fired=tuple(fired))

    def _step_hold(self, entry: int, frame: PadFrame, now: float, blocked: bool) -> bool:
        index = self._index[entry]
        if not frame.buttons[index]:
            self._start[entry] = None
            self._fired[entry] = False
            return False
        if blocked:
            self._start[entry] = None
            return False
        if frame.presses[index]:
            # released and pressed again since the last tick: a new hold
            self._start[entry] = now
            self._fired[entry] = False
            return False
        if self._start[entry] is None:
            self._start[entry] = now
            return False
        if self._fired[entry] or now - self._start[entry] < self._hold_s[entry]:
            return False
        self._fired[entry] = True
        return self.holds[entry].enabled


class HelpCombo:
    """Edge detector of ``help_combo``: True on the tick all its controls become pressed."""

    def __init__(self, layout: GamepadLayout, controls: Sequence[str]) -> None:
        self._axes = tuple(layout.axis_index(n) for n in controls if n in layout.axes)
        self._buttons = tuple(layout.button_index(n) for n in controls if n in layout.buttons)
        self._enabled = bool(controls)
        self._was_on = False

    def step(self, frame: PadFrame) -> bool:
        if not self._enabled:
            return False
        on = all(frame.axes[i] <= HELP_AXIS_PRESSED for i in self._axes) and all(
            frame.buttons[i] for i in self._buttons
        )
        edge = on and not self._was_on
        self._was_on = on
        return edge


@dataclass(frozen=True)
class HelpEntry:
    """One row of the help: the controls (layout names), what they do, the keys."""

    controls: tuple[str, ...]
    function: str
    keys: str = ""


def help_entries(config: TeleopConfig) -> tuple[HelpEntry, ...]:
    """The help rows: labelled controls, then the button roles, then help itself.

    Role texts come from the config; a control's consumer meaning only from
    ``help_labels`` (no label, no row). ``keys`` are the virtual pad's keys.
    """
    button_keys: dict[str, list[str]] = {}
    for entry in config.key_buttons:
        target, _, key = entry.partition(":")
        button_keys.setdefault(target, []).append(key)
    axis_keys: dict[str, list[str]] = {}
    for entry in config.key_axes:
        target, _, key = entry.partition(":")
        axis_keys.setdefault(target[:-1], []).append(f"{key} ({target[-1]})")

    def keys_of(name: str) -> str:
        return " / ".join(button_keys.get(name, []) + axis_keys.get(name, []))

    rows = []
    for entry in config.help_labels:
        name, _, text = entry.partition(":")
        rows.append(HelpEntry((name,), text.strip(), keys_of(name)))
    rows.append(
        HelpEntry(
            (config.deadman_button,),
            "deadman: hold to command; the hold buttons need it released",
            keys_of(config.deadman_button),
        )
    )
    rows.append(HelpEntry((config.estop_button,), "E-STOP", keys_of(config.estop_button)))
    for hold in hold_entries(config):
        seconds = config.reset_hold_s if hold.kind == "reset" else config.hold_s
        if hold.kind == "reset":
            if not hold.enabled:
                continue
            text = f"reset_fault of {hold.target}"
        elif hold.kind == "switch":
            text = f"switch to {hold.target}"
        elif hold.kind == "clear":
            text = f"clear E-STOP ({config.clear_estop_reason} only; E-STOP released)"
        else:
            text = f"call {hold.target}"
        rows.append(
            HelpEntry((hold.button,), f"{text} — hold {seconds:g} s", keys_of(hold.button))
        )
    if config.help_combo or config.help_key:
        rows.append(HelpEntry(tuple(config.help_combo), "show / hide this help", config.help_key))
    return tuple(rows)


LEVEL_NAMES = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}
# A status topic silent this long is shown STALE, whatever its last level said.
STATUS_STALE_S = 2.0


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

    def is_stale(self, now: float) -> bool:
        return now - self.received_s > STATUS_STALE_S


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
