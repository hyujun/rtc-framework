"""Pull-force estimate presentation state for demo_controller_gui (#234 PR-C).

The GUI used to keep the pull block as six loose instance attributes written
field-by-field from the rclpy executor thread and read field-by-field from the
Tk thread. Nothing tied those six reads to one another, so a render could mix
values from two different ticks, and there was no record of *when* the values
arrived — a controller that stopped publishing kept showing its last numbers
forever (#234 P-2/P-8/P-10).

This module holds that state as one immutable ``PullSnapshot``. The callback
builds a whole snapshot and rebinds a single attribute (an atomic rebind under
the GIL); the renderer reads that attribute once into a local. Neither side can
observe a half-updated tick, and the snapshot carries its own arrival time and
source so freshness and provenance are properties of the data rather than
guesses made at render time.

Everything here is pure Python — no Tk, no rclpy Node — so the render state
machine is unit-testable without a display or a ROS graph (matching
``demo_gui.catalog`` / ``demo_gui.config``).

Public surface (imported by app.py):
- PullSnapshot, PULL_UNAVAILABLE, PULL_STALE_AFTER_S
- PullPeakHold
- BadgeState, badge_state, BADGE_* text constants
- basis_label, invalid_reason_label, mask_label, source_label
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from rtc_msgs.msg import PullEstimate

# ── Freshness ────────────────────────────────────────────────────────────────
#
# The controller-owned GraspState/WbcState publishers are woken per RT tick by
# the CM's non-RT publish lane (eventfd, coalescing — see
# rtc_controller_manager/src/rt_controller_node_publish.cpp), so the wire rate
# tracks control_rate and is always far above the GUI's own 5 Hz redraw. The
# threshold is therefore set against the *GUI* period, not control_rate: 0.5 s
# is 2.5 render frames, long enough that a coalesced burst or a scheduling hiccup
# never flickers the badge, short enough that a controller that stopped
# publishing (deactivated, crashed, switched away) is called out within three
# frames instead of showing stale numbers indefinitely.
PULL_STALE_AFTER_S = 0.5

# ── Source labels ────────────────────────────────────────────────────────────
#
# Which parent message fed the snapshot. joint/task controllers publish
# GraspState only and the WBC controller publishes WbcState only (see
# {joint,task,wbc}/lifecycle.cpp Setup*StatePublisher), and the GUI binds both
# subscriptions under the *active* controller's namespace — so exactly one of
# these is live at a time. The label exists to make that visible, not to
# arbitrate between two concurrent writers.
SOURCE_NONE = ""
SOURCE_GRASP = "grasp_state"
SOURCE_WBC = "wbc_state"

_SOURCE_LABELS = {
    SOURCE_NONE: "--",
    SOURCE_GRASP: "GraspState",
    SOURCE_WBC: "WbcState",
}

# ── Wire enum labels ─────────────────────────────────────────────────────────
#
# Keyed off the PullEstimate constants rather than bare literals so a renumber
# in the .msg cannot silently repoint a label.
_BASIS_LABELS = {
    PullEstimate.BASIS_NONE: "none",
    PullEstimate.BASIS_REFERENCE: "reference",
    PullEstimate.BASIS_CARRY: "carry",
    PullEstimate.BASIS_SEED: "seed",
}

_INVALID_LABELS = {
    PullEstimate.INVALID_NONE: "valid",
    PullEstimate.INVALID_NOT_INITIALIZED: "not initialized",
    PullEstimate.INVALID_DEGENERATE_NORMAL: "degenerate normal",
    PullEstimate.INVALID_REQUIRED_CONTACT: "required contact lost",
    PullEstimate.INVALID_INSUFFICIENT_CONTACTS: "too few contacts",
}


def basis_label(basis_source: int) -> str:
    """Human label for ``PullEstimate.basis_source``.

    Only ``BASIS_REFERENCE`` means ``force_inplane[0]`` is the component along
    the configured reference direction; the fallbacks are path-dependent, which
    is why the GUI shows the rule rather than naming the in-plane axes.
    """
    return _BASIS_LABELS.get(basis_source, f"? ({basis_source})")


def invalid_reason_label(invalid_reason: int) -> str:
    """Human label for ``PullEstimate.invalid_reason`` — the first validity gate
    that failed. This is what lets the UNKNOWN badge carry a cause instead of
    collapsing every failure into a bare ``valid == False``."""
    return _INVALID_LABELS.get(invalid_reason, f"? ({invalid_reason})")


def source_label(source: str) -> str:
    """Human label for the publishing parent message."""
    return _SOURCE_LABELS.get(source, source or "--")


# What a field shows when its gate does not pass. Deliberately the same glyph
# the panel already used for "no value", so a gated field reads as absent
# rather than as a plausible zero — a pull magnitude of "0.00 N" and one that
# is simply not being estimated must not look alike (#234 P-3).
PLACEHOLDER = "--"


def gated(text: str, ok: bool) -> str:
    """``text`` when its gate passes, ``PLACEHOLDER`` otherwise."""
    return text if ok else PLACEHOLDER


def format_vec(values, nd: int = 2) -> str:
    """Fixed-precision vector for the panel, e.g. ``(0.10, -0.02, 1.00)``."""
    return "(" + ", ".join(f"{v:.{nd}f}" for v in values) + ")"


def mask_label(mask: int, num_bits: int = 4) -> str:
    """Render a contact/touch bitmask as fixed-width contact slots, bit k =
    contact k in the estimator's configured tip order. Bit 0 is drawn leftmost
    so the string reads in the same order as the configured tips (the opposite
    of a plain binary literal)."""
    if num_bits <= 0:
        return "--"
    return "".join("#" if mask & (1 << k) else "." for k in range(num_bits))


# ── Snapshot ─────────────────────────────────────────────────────────────────


@dataclass(frozen=True, slots=True)
class PullSnapshot:
    """One tick of PullEstimate, plus how and when it reached the GUI.

    Frozen so a reference handed to the renderer cannot be mutated underneath
    it: the callback publishes a *new* snapshot rather than editing the live
    one. Every wire field of ``rtc_msgs/msg/PullEstimate`` is carried, so what
    the GUI can display is limited by the panel layout, not by the storage
    (#234 P-8); ``test_demo_gui_pull`` asserts that coverage against the
    message definition so a future ABI addition fails the test rather than
    being silently dropped here.

    ``recv_monotonic`` is ``None`` exactly when no message has been stored —
    the initial state and the post-rewire reset. It is a monotonic clock
    reading (``time.monotonic``), never a wall clock, so age stays correct
    across a system time step. The message ``stamp`` is deliberately *not* used
    for age: it is the controller's clock, and the GUI is asking "is this
    process still hearing from that controller", which is a receiver-side
    question.
    """

    # ── Provenance / freshness (GUI-side, not on the wire) ──
    source: str = SOURCE_NONE
    recv_monotonic: float | None = None
    frame_id: str = ""

    # ── PullEstimate wire fields ──
    force: tuple[float, float, float] = (0.0, 0.0, 0.0)
    force_inplane: tuple[float, float] = (0.0, 0.0)
    plane_normal: tuple[float, float, float] = (0.0, 0.0, 0.0)
    basis_x: tuple[float, float, float] = (0.0, 0.0, 0.0)
    basis_source: int = PullEstimate.BASIS_NONE
    invalid_reason: int = PullEstimate.INVALID_NOT_INITIALIZED
    contact_mask: int = 0
    touch_mask: int = 0
    magnitude: float = 0.0
    directional: float = 0.0
    friction_utilization: float = 0.0
    leakage_bound: float = 0.0
    valid_contact_count: int = 0
    valid: bool = False
    slip_risk: bool = False
    any_saturated: bool = False
    baseline_applied: bool = False

    # -- construction ---------------------------------------------------------

    @classmethod
    def from_message(cls, msg, source: str, now: float) -> PullSnapshot:
        """Build a snapshot from a GraspState/WbcState parent message.

        ``msg.pull`` is the same embedded PullEstimate in both parents (the
        field names were chosen to match the POD so one reader covers both), and
        the reference frame of every vector is the *parent's* ``header.frame_id``
        — the controller's FK root link, filled at publisher setup
        (owned_topics.cpp SetupGraspStatePublisher / SetupWbcStatePublisher).
        """
        pull = msg.pull
        return cls(
            source=source,
            recv_monotonic=now,
            frame_id=msg.header.frame_id,
            force=tuple(float(v) for v in pull.force),
            force_inplane=tuple(float(v) for v in pull.force_inplane),
            plane_normal=tuple(float(v) for v in pull.plane_normal),
            basis_x=tuple(float(v) for v in pull.basis_x),
            basis_source=int(pull.basis_source),
            invalid_reason=int(pull.invalid_reason),
            contact_mask=int(pull.contact_mask),
            touch_mask=int(pull.touch_mask),
            magnitude=float(pull.magnitude),
            directional=float(pull.directional),
            friction_utilization=float(pull.friction_utilization),
            leakage_bound=float(pull.leakage_bound),
            valid_contact_count=int(pull.valid_contact_count),
            valid=bool(pull.valid),
            slip_risk=bool(pull.slip_risk),
            any_saturated=bool(pull.any_saturated),
            baseline_applied=bool(pull.baseline_applied),
        )

    @staticmethod
    def unavailable() -> PullSnapshot:
        """The 'nothing has been received' snapshot — initial state and the
        value ``_rewire_owned_topics`` resets to, so a controller switch cannot
        leave the previous source's numbers on screen (#234 P-10)."""
        return PULL_UNAVAILABLE

    # -- freshness ------------------------------------------------------------

    @property
    def received(self) -> bool:
        return self.recv_monotonic is not None

    def age_s(self, now: float) -> float | None:
        """Seconds since arrival, or ``None`` if nothing has arrived. Clamped at
        zero so a non-monotonic caller (a test passing an earlier ``now``) can
        never render a negative age."""
        if self.recv_monotonic is None:
            return None
        return max(0.0, now - self.recv_monotonic)

    def is_stale(self, now: float, stale_after_s: float = PULL_STALE_AFTER_S) -> bool:
        """True when nothing has arrived, or the last arrival is older than the
        threshold. 'Never received' counts as stale so callers get one predicate
        for 'do not trust these numbers'; ``received`` distinguishes the two."""
        age = self.age_s(now)
        return age is None or age > stale_after_s


PULL_UNAVAILABLE = PullSnapshot()


# ── Peak-hold ────────────────────────────────────────────────────────────────


@dataclass(slots=True)
class PullPeakHold:
    """Between-frame extremes for the two slip diagnostics (#234 P-20).

    The wire rate follows control_rate (hundreds of Hz) while the panel redraws
    at 5 Hz, so sampling the newest snapshot at render time shows roughly one
    tick in a hundred — exactly the ticks where a transient slip spike is
    invisible. Every stored snapshot is folded in here instead, and the render
    consumes and clears the accumulator, so the panel reports the worst case
    seen over the frame rather than an arbitrary sample of it.

    Deliberately *not* a decaying hold: the value shown always belongs to the
    frame just ended, so a spike is visible for one frame and then gone, and a
    quiet frame reads quiet. ``reset`` drops the accumulator entirely, which is
    what a stale/unavailable stream needs — otherwise a peak from before a
    controller switch or a publish gap would outlive its source.
    """

    friction_utilization: float = 0.0
    slip_risk: bool = False
    samples: int = 0

    def observe(self, snap: PullSnapshot) -> None:
        """Fold one stored snapshot into the frame's extremes. Called from the
        rclpy executor thread on every message."""
        self.friction_utilization = max(self.friction_utilization, snap.friction_utilization)
        self.slip_risk = self.slip_risk or snap.slip_risk
        self.samples += 1

    def reset(self) -> None:
        self.friction_utilization = 0.0
        self.slip_risk = False
        self.samples = 0

    def consume(self) -> tuple[float, bool, int]:
        """Return ``(peak_friction_utilization, any_slip_risk, samples)`` for the
        frame and clear. ``samples == 0`` means no message arrived during the
        frame — the caller falls back to the snapshot's own values rather than
        reporting a zero peak that never happened."""
        out = (self.friction_utilization, self.slip_risk, self.samples)
        self.reset()
        return out

    def merge_into(self, snap: PullSnapshot) -> PullSnapshot:
        """Project the frame's extremes onto ``snap`` for rendering.

        Returns ``snap`` unchanged when no sample arrived this frame (nothing
        was observed, so there is no peak to report) or when the snapshot is
        the unavailable one. Only the two peak-held diagnostics are replaced;
        every other field stays as the newest tick left it, so the panel never
        mixes a peak-held value into a vector that must stay tick-coherent.
        """
        util, slip, samples = self.consume()
        if samples == 0 or not snap.received:
            return snap
        return replace(
            snap,
            friction_utilization=max(snap.friction_utilization, util),
            slip_risk=snap.slip_risk or slip,
        )


# ── Badge state machine ──────────────────────────────────────────────────────

BADGE_SLIP = "SLIP RISK"
BADGE_NO_SLIP = "NO SLIP RISK"
BADGE_UNKNOWN = "UNKNOWN"
BADGE_STALE = "STALE"
BADGE_UNAVAILABLE = "UNAVAILABLE"

_COLOR_RED = ("#f38ba8", "#1e1e2e")
_COLOR_GREY = ("#585b70", "#cdd6f4")
_COLOR_AMBER = ("#f9e2af", "#1e1e2e")
_COLOR_DARK = ("#45475a", "#a6adc8")


@dataclass(frozen=True, slots=True)
class BadgeState:
    """What the slip badge shows: ``text`` is the badge itself, ``detail`` the
    adjacent one-line cause (empty when the badge is self-explanatory)."""

    text: str
    bg: str
    fg: str
    detail: str = ""

    @property
    def key(self) -> str:
        """Dirty-check key for the Tk redraw cache — covers everything the
        widget renders, so a detail-only change still repaints."""
        return f"{self.text}|{self.bg}|{self.fg}|{self.detail}"


def _badge(text: str, colors: tuple[str, str], detail: str = "") -> BadgeState:
    bg, fg = colors
    return BadgeState(text=text, bg=bg, fg=fg, detail=detail)


def badge_state(
    snap: PullSnapshot,
    now: float,
    *,
    estop_active: bool = False,
    stale_after_s: float = PULL_STALE_AFTER_S,
) -> BadgeState:
    """Resolve the pull badge.

    Priority, highest first:

    1. fresh + ``slip_risk``  → SLIP RISK
    2. fresh + ``valid``      → NO SLIP RISK
    3. fresh + ``!valid``     → UNKNOWN (+ ``invalid_reason`` as the cause)
    4. stale / never received / E-STOP → STALE / UNAVAILABLE

    The three positive states share ``fresh`` as a precondition, so freshness is
    resolved first; within a fresh tick, slip risk outranks validity because
    ``slip_risk`` and ``friction_utilization`` are evaluated outside the
    ``valid`` gate by the estimator itself (#234 P-7) — tips that are still
    holding must be able to report slip while the pull *vector* is invalid.

    E-STOP is resolved with the bottom bucket rather than as a fresh tick even
    though the controller keeps publishing through it (#234 P-1 guarantees a
    row every tick). ``StageEstopPullTick`` drives the estimator with an
    all-invalid input span and a zero normal, so an E-STOP tick reports
    ``valid == False`` with ``INVALID_DEGENERATE_NORMAL`` — reading that as
    "UNKNOWN: degenerate normal" would name a geometry failure that did not
    happen. No information is suppressed by the override: that same path clears
    every contact, so ``slip_risk`` is false for the duration.
    """
    if estop_active:
        return _badge(BADGE_UNAVAILABLE, _COLOR_DARK, "E-STOP active")

    if snap.is_stale(now, stale_after_s):
        if not snap.received:
            return _badge(BADGE_UNAVAILABLE, _COLOR_DARK, "no estimate received")
        age = snap.age_s(now) or 0.0
        return _badge(BADGE_STALE, _COLOR_AMBER, f"last update {age:.1f} s ago")

    if snap.slip_risk:
        return _badge(BADGE_SLIP, _COLOR_RED)
    if snap.valid:
        return _badge(BADGE_NO_SLIP, _COLOR_GREY)
    return _badge(BADGE_UNKNOWN, _COLOR_AMBER, invalid_reason_label(snap.invalid_reason))


# ── Panel render ─────────────────────────────────────────────────────────────

# Value-label foreground; individual fields override to flag a state.
VALUE_FG = "#f9e2af"
_FG_OK = "#a6e3a1"
_FG_ALERT = "#f38ba8"

# Field keys, grouped by the gate that governs them. app.py builds one widget
# per key and blits ``PullRender.values`` onto them, so adding a field here and
# in the panel spec is the whole change — the gates stay in one place.
VECTOR_FIELDS = (
    "valid",
    "magnitude",
    "directional",
    "leakage",
    "force",
    "inplane",
    "basis",
    "basis_x",
)
DIAGNOSTIC_FIELDS = (
    "friction_util",
    "contact_set",
    "touch_set",
    "saturated",
    "baseline",
    "plane_normal",
)
SOURCE_FIELDS = ("source", "frame")


@dataclass(frozen=True, slots=True)
class PullRender:
    """Everything the panel draws for one frame."""

    badge: BadgeState
    values: dict[str, str]
    colors: dict[str, str]
    trusted: bool
    vector_ok: bool


def build_render(
    snap: PullSnapshot,
    now: float,
    *,
    estop_active: bool = False,
    num_contacts: int = 4,
    stale_after_s: float = PULL_STALE_AFTER_S,
) -> PullRender:
    """Resolve badge + every field's text for one render frame.

    Two gates, deliberately different (#234 P-3/P-7):

    - ``trusted`` — the badge does not read STALE/UNAVAILABLE. When it fails,
      every value blanks: numbers from a publisher that stopped, or from before
      a controller switch, are worse than no numbers.
    - ``vector_ok`` — additionally ``valid``, which is *pull-vector* validity
      only. It gates magnitude / force / in-plane / directional / leakage / the
      basis. It does **not** gate the contact diagnostics, because the estimator
      evaluates friction utilisation and slip risk outside that gate — a
      required-role dropout must still report slip for the tips still holding.

    ``trusted`` is derived from the badge rather than recomputed beside it, so
    the panel can never show live-looking numbers under a STALE header.

    ``plane_normal`` sits in the diagnostic group on purpose: per
    PullEstimate.msg a tick can carry a usable normal but no basis (the plane
    was fine, a required tip dropped out), so gating it on ``valid`` would hide
    a value the wire says is good.
    """
    badge = badge_state(snap, now, estop_active=estop_active, stale_after_s=stale_after_s)
    trusted = badge.text not in (BADGE_STALE, BADGE_UNAVAILABLE)
    vector_ok = trusted and snap.valid

    age = snap.age_s(now)
    values = {
        # ── Pull vector (valid-gated) ──
        "valid": gated("OK" if snap.valid else invalid_reason_label(snap.invalid_reason), trusted),
        "magnitude": gated(f"{snap.magnitude:.2f} N", vector_ok),
        "directional": gated(f"{snap.directional:.2f} N", vector_ok),
        "leakage": gated(f"{snap.leakage_bound:.2f} N", vector_ok),
        "force": gated(format_vec(snap.force), vector_ok),
        "inplane": gated(format_vec(snap.force_inplane), vector_ok),
        # The rule, not an axis name: force_inplane[0] is the component along
        # the configured reference direction only under BASIS_REFERENCE; the
        # fallbacks are path-dependent and carry no such naming (#234 P-11).
        "basis": gated(basis_label(snap.basis_source), vector_ok),
        "basis_x": gated(format_vec(snap.basis_x), vector_ok),
        # ── Contact diagnostics (outside the valid gate) ──
        "friction_util": gated(f"{snap.friction_utilization:.2f}", trusted),
        "contact_set": gated(
            f"{mask_label(snap.contact_mask, num_contacts)} ({snap.valid_contact_count})", trusted
        ),
        "touch_set": gated(mask_label(snap.touch_mask, num_contacts), trusted),
        "saturated": gated("YES" if snap.any_saturated else "no", trusted),
        "baseline": gated("ON" if snap.baseline_applied else "off", trusted),
        "plane_normal": gated(format_vec(snap.plane_normal), trusted),
        # ── Provenance — shown unconditionally: "which controller, how long
        # ago" is exactly what the operator needs when the values are gone. ──
        "source": f"{source_label(snap.source)} · "
        f"{PLACEHOLDER if age is None else f'{age:.2f} s'}",
        "frame": snap.frame_id or PLACEHOLDER,
    }
    colors = {
        "valid": _FG_OK if vector_ok else VALUE_FG,
        "saturated": _FG_ALERT if (trusted and snap.any_saturated) else VALUE_FG,
    }
    return PullRender(
        badge=badge, values=values, colors=colors, trusted=trusted, vector_ok=vector_ok
    )
