"""Pure model of the hand joint-command GUI (no rclpy, no Qt).

The GUI (``joint_command_gui``) drives one or more hands that each publish
``sensor_msgs/JointState`` on ``<ns>/<side>/joint_states`` and consume
``rtc_msgs/JointCommand`` on ``<ns>/<side>/joint_command`` (a node without
sides uses ``<ns>/joint_states`` / ``<ns>/joint_command``). Everything that
decides WHAT is published lives here so that it can be tested with a fake
clock; ``joint_command_ros`` only moves messages and ``joint_command_view``
only draws.

Contract of one hand (``HandController``):

- The joint count and names come from the first ``joint_states`` sample and
  are then fixed. A later sample with other names (or another count) blocks
  Enable and disables a running hand.
- Units: the GUI works in degrees; ``tick`` returns radians in the fixed
  joint-name order, in the ``joint_states`` frame (the hand node applies its
  own per-joint offset to the command).
- Enable starts OFF. Enabling syncs both the target and the published command
  to the latest measured position, so the first published value is where the
  hand already is. It is refused unless the sample is fresh, the link is not
  reported down, no other node publishes on the command topic, and every
  measured position is finite and inside its slider range. While disabled,
  ``status_line`` says what blocks Enable at that moment.
- While enabled, ``tick`` moves the command toward the target at most
  ``slew_rate_deg_s`` per second (0 = no limit) and refuses (disables) on a
  non-finite or out-of-range value. Disable only stops publishing — the hand
  holds its last command.
- Auto-disable: ``joint_states`` older than ``stale_timeout_s``, link status
  false, or another publisher on the command topic. A link status that never
  arrived is "unknown" and does not disable on its own.
- Ranges: YAML ``ranges_deg`` are the initial per-joint ranges; a joint
  without one gets ``DEFAULT_RANGE_HALF_DEG`` around its first measured
  position. Ranges are editable at runtime (not saved) and never exceed
  ``ABS_LIMIT_DEG``. While enabled, an edit that would leave the current
  target or command outside the new range is rejected.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path

import yaml

# Hard cap on every slider range (deg), whatever the YAML or the user says.
ABS_LIMIT_DEG = 180.0
# Half-width (deg) of the range a joint gets when the YAML names none.
DEFAULT_RANGE_HALF_DEG = 10.0
# Label of the hand served by a node without sides (`<ns>/joint_states`).
SINGLE_HAND_LABEL = "single"
# Longest step (s) one tick integrates the slew over — or two publish periods,
# if longer: a GUI stall must not turn into one large jump when it resumes.
MAX_TICK_DT_S = 0.1
# publish_rate_hz bounds: the publish QTimer takes whole milliseconds.
MIN_PUBLISH_RATE_HZ = 1.0
MAX_PUBLISH_RATE_HZ = 1000.0

_CONFIG_KEYS = {
    "namespace",
    "sides",
    "publish_rate_hz",
    "slew_rate_deg_s",
    "stale_timeout_ms",
    "ranges_deg",
}


class ConfigError(ValueError):
    """The GUI YAML is malformed or out of range."""


@dataclass(frozen=True)
class GuiConfig:
    namespace: str = "/hand"
    sides: tuple[str, ...] = ("right", "left")
    publish_rate_hz: float = 50.0
    slew_rate_deg_s: float = 90.0
    stale_timeout_s: float = 0.1
    # hand label -> joint name -> (min_deg, max_deg)
    ranges_deg: Mapping[str, Mapping[str, tuple[float, float]]] = field(default_factory=dict)


@dataclass(frozen=True)
class HandTopics:
    label: str
    state_topic: str
    command_topic: str
    link_topic: str


@dataclass(frozen=True)
class HandSample:
    """One ``joint_states`` message, stamped with the receive time."""

    names: tuple[str, ...]
    position_rad: tuple[float, ...]
    effort: tuple[float, ...]
    received_s: float


def _number(value, what: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ConfigError(f"{what} must be a number, got {value!r}")
    value = float(value)
    if not math.isfinite(value):
        raise ConfigError(f"{what} must be finite, got {value!r}")
    return value


def _range(value, what: str) -> tuple[float, float]:
    if not isinstance(value, list | tuple) or len(value) != 2:
        raise ConfigError(f"{what} must be [min, max] in degrees, got {value!r}")
    lo = _number(value[0], f"{what}[0]")
    hi = _number(value[1], f"{what}[1]")
    reason = range_error(lo, hi)
    if reason:
        raise ConfigError(f"{what}: {reason}")
    return lo, hi


def range_error(lo: float, hi: float) -> str | None:
    """Why ``[lo, hi]`` (deg) is not a valid slider range, or None."""
    if not (math.isfinite(lo) and math.isfinite(hi)):
        return "min and max must be finite"
    if lo >= hi:
        return f"min {lo:g} must be below max {hi:g}"
    if lo < -ABS_LIMIT_DEG or hi > ABS_LIMIT_DEG:
        return f"range must stay within ±{ABS_LIMIT_DEG:g} deg"
    return None


def normalize_namespace(namespace) -> str:
    """``/a/b/`` -> ``/a/b``; anything but an absolute name raises ConfigError."""
    if not isinstance(namespace, str) or not namespace.startswith("/"):
        raise ConfigError(f"namespace must be an absolute name like /hand, got {namespace!r}")
    return namespace.rstrip("/") or "/"


def parse_config(data) -> GuiConfig:
    """Build a ``GuiConfig`` from the parsed YAML mapping (None = defaults)."""
    if data is None:
        return GuiConfig()
    if not isinstance(data, dict):
        raise ConfigError("the GUI config must be a mapping")
    unknown = set(data) - _CONFIG_KEYS
    if unknown:
        raise ConfigError(f"unknown config keys: {sorted(unknown)}")
    default = GuiConfig()

    namespace = normalize_namespace(data.get("namespace", default.namespace))

    sides = data.get("sides", list(default.sides))
    if (
        not isinstance(sides, list)
        or not all(isinstance(s, str) and s and "/" not in s for s in sides)
        or len(set(sides)) != len(sides)
    ):
        raise ConfigError(f"sides must be a list of distinct names, got {sides!r}")
    if SINGLE_HAND_LABEL in sides:
        raise ConfigError(f"'{SINGLE_HAND_LABEL}' is reserved for a node without sides")

    rate = _number(data.get("publish_rate_hz", default.publish_rate_hz), "publish_rate_hz")
    if not MIN_PUBLISH_RATE_HZ <= rate <= MAX_PUBLISH_RATE_HZ:
        raise ConfigError(
            f"publish_rate_hz must be in [{MIN_PUBLISH_RATE_HZ:g}, {MAX_PUBLISH_RATE_HZ:g}]"
        )
    slew = _number(data.get("slew_rate_deg_s", default.slew_rate_deg_s), "slew_rate_deg_s")
    if slew < 0.0:
        raise ConfigError("slew_rate_deg_s must be >= 0 (0 = no limit)")
    stale_ms = _number(
        data.get("stale_timeout_ms", default.stale_timeout_s * 1000.0), "stale_timeout_ms"
    )
    if stale_ms <= 0.0:
        raise ConfigError("stale_timeout_ms must be > 0")

    ranges_in = data.get("ranges_deg") or {}
    if not isinstance(ranges_in, dict):
        raise ConfigError("ranges_deg must map a hand label to {joint: [min, max]}")
    labels = set(sides) | {SINGLE_HAND_LABEL}
    ranges: dict[str, dict[str, tuple[float, float]]] = {}
    for label, joints in ranges_in.items():
        if label not in labels:
            raise ConfigError(f"ranges_deg: unknown hand {label!r} (known: {sorted(labels)})")
        joints = joints or {}
        if not isinstance(joints, dict):
            raise ConfigError(f"ranges_deg.{label} must map joint name to [min, max]")
        ranges[label] = {
            str(name): _range(value, f"ranges_deg.{label}.{name}")
            for name, value in joints.items()
        }

    return GuiConfig(
        namespace=namespace,
        sides=tuple(sides),
        publish_rate_hz=rate,
        slew_rate_deg_s=slew,
        stale_timeout_s=stale_ms / 1000.0,
        ranges_deg=ranges,
    )


def load_config(path: str | Path | None) -> GuiConfig:
    """Read the GUI YAML at ``path`` (empty / None = all defaults)."""
    if not path:
        return GuiConfig()
    try:
        with open(path, encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
    except OSError as exc:
        raise ConfigError(f"cannot read {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ConfigError(f"{path} is not valid YAML: {exc}") from exc
    return parse_config(data)


def hand_topics(config: GuiConfig) -> list[HandTopics]:
    """Candidate hands to look for: one per side, then the side-less node."""
    base = "" if config.namespace == "/" else config.namespace

    def topics(label: str, prefix: str) -> HandTopics:
        return HandTopics(
            label=label,
            state_topic=f"{prefix}/joint_states",
            command_topic=f"{prefix}/joint_command",
            link_topic=f"{prefix}/link_status",
        )

    found = [topics(side, f"{base}/{side}") for side in config.sides]
    found.append(topics(SINGLE_HAND_LABEL, base))
    return found


class HandController:
    """Enable state, ranges, slew and safety checks of one hand."""

    def __init__(
        self,
        label: str,
        config: GuiConfig,
    ) -> None:
        self.label = label
        self._slew_deg_s = config.slew_rate_deg_s
        self._stale_s = config.stale_timeout_s
        self._max_dt_s = max(MAX_TICK_DT_S, 2.0 / config.publish_rate_hz)
        self._yaml_ranges = dict(config.ranges_deg.get(label, {}))

        self.joint_names: tuple[str, ...] = ()
        self._initial_ranges: list[tuple[float, float]] = []
        self.ranges: list[tuple[float, float]] = []
        self.target_deg: list[float] = []
        self.command_deg: list[float] = []

        self.latest: HandSample | None = None
        self.link_ok: bool | None = None
        self.foreign_publisher = False
        self.layout_mismatch: str | None = None

        self.enabled = False
        self.status = "waiting for joint_states"
        self._last_tick_s: float | None = None

    # ── inputs ────────────────────────────────────────────────────────────

    @property
    def ready(self) -> bool:
        return bool(self.joint_names)

    def on_sample(self, sample: HandSample) -> None:
        if len(sample.position_rad) != len(sample.names):
            self.layout_mismatch = (
                f"joint_states has {len(sample.names)} names but "
                f"{len(sample.position_rad)} positions"
            )
            self._auto_disable(self.layout_mismatch)
            return
        if not self.joint_names:
            self._lock_layout(sample)
        elif sample.names != self.joint_names:
            self.layout_mismatch = (
                "joint_states names changed since the GUI started — restart the GUI"
            )
            self._auto_disable(self.layout_mismatch)
            return
        self.layout_mismatch = None
        self.latest = sample
        if not self.enabled and self.status == "waiting for joint_states":
            self.status = "disabled"

    def set_link(self, ok: bool | None) -> None:
        self.link_ok = ok
        if ok is False:
            self._auto_disable("link_status is false")

    def set_foreign_publisher(self, present: bool) -> None:
        self.foreign_publisher = present
        if present:
            self._auto_disable("another node publishes joint_command")

    def _lock_layout(self, sample: HandSample) -> None:
        self.joint_names = sample.names
        ranges = []
        for name, rad in zip(sample.names, sample.position_rad, strict=True):
            if name in self._yaml_ranges:
                ranges.append(self._yaml_ranges[name])
                continue
            # Centred on the measurement, slid inward to stay under the cap (a
            # measurement past the cap then lies outside and blocks Enable).
            half = DEFAULT_RANGE_HALF_DEG
            deg = math.degrees(rad) if math.isfinite(rad) else 0.0
            centre = min(max(deg, -ABS_LIMIT_DEG + half), ABS_LIMIT_DEG - half)
            ranges.append((centre - half, centre + half))
        self._initial_ranges = list(ranges)
        self.ranges = list(ranges)
        self.target_deg = [
            min(max(math.degrees(r) if math.isfinite(r) else 0.0, lo), hi)
            for r, (lo, hi) in zip(sample.position_rad, ranges, strict=True)
        ]
        self.command_deg = list(self.target_deg)

    def unconfigured_joints(self) -> list[str]:
        """Joints whose range came from the measured position, not the YAML."""
        return [n for n in self.joint_names if n not in self._yaml_ranges]

    def unknown_yaml_joints(self) -> list[str]:
        """YAML range entries that name no joint of this hand."""
        return sorted(set(self._yaml_ranges) - set(self.joint_names)) if self.ready else []

    # ── user actions ─────────────────────────────────────────────────────

    def measured_deg(self) -> list[float] | None:
        if self.latest is None:
            return None
        return [math.degrees(r) for r in self.latest.position_rad]

    def enable_blocker(self, now_s: float) -> str | None:
        """Why Enable would be refused now, or None."""
        if self.layout_mismatch:
            return self.layout_mismatch
        if not self.ready or self.latest is None:
            return "no joint_states received yet"
        if now_s - self.latest.received_s > self._stale_s:
            return "joint_states is stale"
        if self.link_ok is False:
            return "link_status is false"
        if self.foreign_publisher:
            return "another node publishes joint_command"
        for name, deg, (lo, hi) in zip(
            self.joint_names, self.measured_deg(), self.ranges, strict=True
        ):
            if not math.isfinite(deg):
                return f"{name}: measured position is not finite"
            if not lo <= deg <= hi:
                return f"{name}: measured {deg:.1f} deg is outside the range [{lo:g}, {hi:g}]"
        return None

    def request_enable(self, now_s: float) -> str | None:
        """Enable and sync to the measured position; returns the refusal, or None."""
        blocker = self.enable_blocker(now_s)
        if blocker:
            return blocker
        self.target_deg = self.measured_deg()
        self.command_deg = list(self.target_deg)
        self.enabled = True
        self._last_tick_s = now_s
        self.status = "enabled"
        return None

    def status_line(self, now_s: float) -> str:
        """The last state change and, while disabled, what blocks Enable now."""
        blocker = None if self.enabled else self.enable_blocker(now_s)
        return f"{self.status} — Enable blocked: {blocker}" if blocker else self.status

    def disable(self, reason: str = "disabled") -> None:
        self.enabled = False
        self._last_tick_s = None
        self.status = reason

    def _auto_disable(self, reason: str) -> None:
        if self.enabled:
            self.disable(f"auto-disabled: {reason}")

    def set_target(self, index: int, deg: float) -> float | None:
        """Set one slider target (clamped into its range); None if rejected."""
        if not math.isfinite(deg):
            return None
        lo, hi = self.ranges[index]
        self.target_deg[index] = min(max(deg, lo), hi)
        return self.target_deg[index]

    def set_range(self, index: int, lo: float, hi: float) -> str | None:
        """Change one joint's range; returns the refusal, or None."""
        reason = range_error(lo, hi)
        if reason:
            return reason
        if self.enabled:
            for what, value in (
                ("target", self.target_deg[index]),
                ("command", self.command_deg[index]),
            ):
                if not lo <= value <= hi:
                    return f"current {what} {value:.1f} deg would fall outside [{lo:g}, {hi:g}]"
        self.ranges[index] = (lo, hi)
        self.target_deg[index] = min(max(self.target_deg[index], lo), hi)
        if not self.enabled:
            self.command_deg[index] = self.target_deg[index]
        return None

    def reset_ranges(self) -> str | None:
        """Restore every range to its initial value; all-or-nothing."""
        if self.enabled:
            for i, (lo, hi) in enumerate(self._initial_ranges):
                for value in (self.target_deg[i], self.command_deg[i]):
                    if not lo <= value <= hi:
                        return (
                            f"{self.joint_names[i]}: current value {value:.1f} deg would fall "
                            f"outside the initial range [{lo:g}, {hi:g}]"
                        )
        for i, (lo, hi) in enumerate(self._initial_ranges):
            self.set_range(i, lo, hi)
        return None

    # ── 50 Hz tick ───────────────────────────────────────────────────────

    def tick(self, now_s: float) -> list[float] | None:
        """Advance one publish period; the command (rad) to publish, or None."""
        if not self.enabled:
            return None
        if self.latest is None or now_s - self.latest.received_s > self._stale_s:
            self.disable("auto-disabled: joint_states is stale")
            return None
        # The setters already disabled on these; re-checked so that no ordering
        # of inputs and ticks can publish past them.
        if self.link_ok is False or self.foreign_publisher or self.layout_mismatch:
            self.disable("auto-disabled: link down, foreign publisher or layout change")
            return None

        dt = 0.0 if self._last_tick_s is None else now_s - self._last_tick_s
        dt = min(max(dt, 0.0), self._max_dt_s)
        self._last_tick_s = now_s
        step = math.inf if self._slew_deg_s == 0.0 else self._slew_deg_s * dt
        for i, target in enumerate(self.target_deg):
            delta = target - self.command_deg[i]
            self.command_deg[i] += max(-step, min(step, delta))

        for name, deg, (lo, hi) in zip(
            self.joint_names, self.command_deg, self.ranges, strict=True
        ):
            # every range passed range_error, so this also holds the ±ABS_LIMIT_DEG cap
            if not math.isfinite(deg) or not lo <= deg <= hi:
                self.disable(f"auto-disabled: {name} command {deg} deg is invalid")
                return None
        return [math.radians(d) for d in self.command_deg]


def describe_link(ok: bool | None) -> str:
    return {None: "unknown", True: "up", False: "DOWN"}[ok]
