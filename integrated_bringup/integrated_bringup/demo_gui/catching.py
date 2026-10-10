"""Catching controller panel state for demo_controller_gui (dynamic_catching §13 S5).

Pure Python — no Tk, no rclpy — so it is unit-testable without a display or a
ROS graph, following ``demo_gui.ball_launch`` / ``demo_gui.hand_step``.

THREE THINGS THIS PANEL REFUSES TO COLLAPSE.

**Observed arming against requested arming.** ``catching.enable`` is a
parameter the operator writes, but the RT tick LOWERS the latch itself on
E-STOP and on a latched fault (A-S5-3, P-1 (c)). So a successful parameter set
is not proof the controller armed, and the two values disagree exactly when
something disarmed it out from under the operator — which is the case worth
seeing. The panel shows the observed latch as the headline and the requested
one beside it, and says so when they differ.

**A refused input lane against a silent one.** From the controller's side both
look like "no usable prediction". The reject counters are the only thing that
tells them apart, so they are shown whenever any of them is non-zero rather
than hidden behind a details view.

**A feed that never started against one that stopped.** Same three-state
treatment ``ball_launch`` uses, and for the same reason: during bring-up those
two look identical on screen while meaning opposite things.

WHAT IS NOT SHOWN AS A NUMBER. Any block the controller did not compute this
tick arrives zeroed with its ``*_valid`` companion false (PROC-7). The panel
renders those as ``--`` rather than as 0, because a zero reference and an
absent reference are different facts and only one of them is a measurement.

Public surface (imported by app.py):
- CATCHING_CONFIG_KEY, CATCHING_ENABLE_PARAM, CATCHING_STATE_TOPIC
- CATCHING_SEGMENT_MODE_PARAM, SEGMENT_MODE_QUERY_PERIOD_S, SEGMENT_MODE_REPLY_TIMEOUT_S,
  segment_mode_query_due
- CATCHING_SEARCH_MODE_PARAM, search_mode_query_due (same throttle, `planner.search.mode`)
- budget_param_names, segment_budget_query_due, search_budget_query_due (the budgets of the
  selected planner, read once the mode is known)
- MODE_NAMES, REASON_NAMES, PLAN_REASON_NAMES
- CatchingStatus
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .ball_launch import PLACEHOLDER, FeedState, FeedStatus

# The controller's config_key, which is also its ROS namespace (see
# demo_gui.hand_step, which targets the same controller).
CATCHING_CONFIG_KEY = "demo_catching_controller"

# The read-write arming parameter (A-S5-3). Not a message or a service: a new
# msg/srv for something a parameter already expresses would be an E-3 decision.
CATCHING_ENABLE_PARAM = "catching.enable"

# The read-only parameter naming the arm-reference planner (`closed_form` | `mpc` |
# `mpc_docking`). It is a parameter rather than a CatchingState field because the
# message is frozen; it is declared only after a successful configure, so a
# parked or unconfigured controller answers with an empty string.
CATCHING_SEGMENT_MODE_PARAM = "planner.segment.mode"
# The catch-point search choice ({grid, nlp}), mirrored beside it. Read with the
# same throttle; a controller built before the key existed never declares it.
CATCHING_SEARCH_MODE_PARAM = "planner.search.mode"

# Minimum spacing between reads of that parameter while it is still unknown.
SEGMENT_MODE_QUERY_PERIOD_S = 2.0
# A read with no reply after this long is taken as lost and asked again: a
# service reply that never arrives leaves its future pending forever.
SEGMENT_MODE_REPLY_TIMEOUT_S = 10.0

# Relative to the controller's namespace — the controller publishes it under
# its own node namespace rather than under a device group's, because it
# describes the CONTROLLER and both claimed groups would have an equal claim
# to the prefix.
CATCHING_STATE_TOPIC = "catching_state"

# rtc::catching::Mode, in the enum's own order (rtc_msgs/CatchingState MODE_*).
MODE_NAMES = (
    "IDLE",
    "ARMED",
    "TRACKING",
    "APPROACH",
    "COMMITTED",
    "CLOSING",
    "DECEL",
    "HOLD",
    "RETREAT",
    "ABORT_SAFE",
    "FAULT",
)

# rtc::catching::Reason (rtc_msgs/CatchingState REASON_*).
REASON_NAMES = (
    "NONE",
    "BALL_STALE",
    "BALL_STALE_COMMITTED",
    "BALL_STALE_LONG",
    "TRACK_CHANGED",
    "HORIZON_EXTRAP",
    "PRED_INCONSISTENT",
    "NO_CATCHABLE_PLAN",
    "PLAN_INVALID",
    "QP_FAILED",
    "REF_SATURATED",
    "JOINT_CONFLICT",
    "TRACK_ERR",
    "ABORT_ESCALATED",
    "ESTOP",
    "FAULT_RESET",
    "SPEED_SCALING",
    "CLOCK_UNHEALTHY",
    "PARAMS_TBD",
    "HAND_TIMEOUT",
    "TIP_STALE",
)

# rtc::catching::PlanReason (rtc_msgs/CatchingState PLAN_REASON_*): why there is
# no plan — the planner's first bottleneck (dynamic_catching S6, decision E).
PLAN_REASON_NAMES = (
    "NONE",
    "UNCERTAINTY",
    "IK_FAILED",
    "MANIPULABILITY",
    "REACH_TIME",
    "LIMITS_INVALID",
    "GAMMA_WINDOW",
    "STOPPING_DISTANCE",
    "ROLLOUT",
    "ERROR_BUDGET",
    "IMPULSE",
    "HORIZON_SHORT",
    "BUDGET_EXCEEDED",
    "INPUT_NON_FINITE",
)

# rtc::catching::Outcome (rtc_msgs/CatchingState OUTCOME_*): how the LAST
# attempt ended (S7.3). Kept across a re-arm by the controller.
OUTCOME_NAMES = (
    "NONE",
    "CAPTURED",
    "MISSED",
    "UNDETERMINED",
    "ABORTED",
)

# rtc::catching::HandPhase (rtc_msgs/CatchingState HAND_PHASE_*), S7.1.
HAND_PHASE_NAMES = (
    "OPEN",
    "PRESHAPE",
    "CLOSE",
    "HOLD",
    "RELEASE",
)

# Modes the operator should be able to spot without reading the word.
_ALARM_MODES = frozenset({"ABORT_SAFE", "FAULT"})

# Histogram buckets that count messages which were not stored but are not a
# defect of the lane either. `no_track` is the publisher's idle state (an empty
# cloud while no ball is in view) and is non-zero on every healthy run, so
# listing it under "input rejects" would make that line permanent and hide the
# counter that moved for a reason. Matched by NAME — the publisher stamps them
# — so a bag from a build without the bucket reads the same way.
_NOT_A_REJECT = frozenset({"no_track"})


def segment_mode_query_due(
    status: CatchingStatus,
    now_s: float,
    last_query_s: float | None,
    in_flight: bool,
) -> bool:
    """Whether the GUI should ask the controller for its segment mode now.

    Throttled because a parked controller answers with an empty value forever:
    an unthrottled retry from the 200 ms refresh would send 5 requests/s
    indefinitely. Nothing is asked before the feed has been received once, and
    nothing once the law is cached. A read still `in_flight` blocks the next
    one only until `SEGMENT_MODE_REPLY_TIMEOUT_S`: past that its reply is lost.
    """
    return _mirror_query_due(status, status.segment_mode, now_s, last_query_s, in_flight)


def search_mode_query_due(
    status: CatchingStatus,
    now_s: float,
    last_query_s: float | None,
    in_flight: bool,
) -> bool:
    """`segment_mode_query_due` for `planner.search.mode`: same throttle, own cache."""
    return _mirror_query_due(status, status.search_mode, now_s, last_query_s, in_flight)


def _mirror_query_due(
    status: CatchingStatus,
    cached: str | None,
    now_s: float,
    last_query_s: float | None,
    in_flight: bool,
) -> bool:
    if status.feed.last_seen_s is None or cached is not None:
        return False
    if last_query_s is None:
        return not in_flight
    waited_s = now_s - last_query_s
    if in_flight:
        return waited_s >= SEGMENT_MODE_REPLY_TIMEOUT_S
    return waited_s >= SEGMENT_MODE_QUERY_PERIOD_S


# Read-only budget mirrors of the SELECTED planner, in the order `_format_budget`
# reads them. Only the selected implementation declares its own (the others are
# "not set"), so nothing is asked before the mode is known. The grid search and
# the closed-form law declare no budget parameter: they have nothing to show.
_SEGMENT_BUDGET_PARAMS = {
    "mpc": (
        "planner.segment.mpc.budget.first_s",
        "planner.segment.mpc.budget.replan_s",
    ),
    "mpc_docking": (
        "planner.segment.mpc_docking.budget.first_s",
        "planner.segment.mpc_docking.budget.replan_s",
    ),
}
_SEARCH_BUDGET_PARAMS = {
    "nlp": (
        "planner.search.nlp.budget.budget_s",
        "planner.search.nlp.budget.solve_s",
        "planner.search.nlp.budget.max_solves",
    ),
}

# Implementations that run only in simulation as shipped (a hardware
# configuration selecting one is parked at configure while
# robot.hand.docking.provisional is true). Stated from the mode name so the
# operator sees it without a query that could fail.
_SIM_ONLY_MODES = frozenset({"nlp", "mpc_docking"})

# Segment modes whose planner publishes a plan only together with its first
# segment, so a withheld segment leaves no plan and no reason on the feed.
_PLAN_WITH_FIRST_SEGMENT_MODES = frozenset({"mpc", "mpc_docking"})

PLAN_WITH_FIRST_SEGMENT_HINT = (
    "  (this planner publishes a plan only with its first segment — a withheld one shows "
    "nothing here; see planner_events.csv: segment_outcome, segment_core_reason)"
)


def budget_param_names(kind: str, mode: str | None) -> tuple[str, ...]:
    """Budget parameters to read for the selected `mode`; `kind` is segment | search.

    Empty when the mode is unknown or declares no budget, which is also the
    answer for a mode this build has never heard of: nothing is guessed.
    """
    table = _SEGMENT_BUDGET_PARAMS if kind == "segment" else _SEARCH_BUDGET_PARAMS
    return table.get(mode, ()) if mode is not None else ()


def segment_budget_query_due(
    status: CatchingStatus,
    now_s: float,
    last_query_s: float | None,
    in_flight: bool,
) -> bool:
    """`segment_mode_query_due` for the selected segment planner's budgets."""
    if not budget_param_names("segment", status.segment_mode):
        return False
    return _mirror_query_due(status, status.segment_budget, now_s, last_query_s, in_flight)


def search_budget_query_due(
    status: CatchingStatus,
    now_s: float,
    last_query_s: float | None,
    in_flight: bool,
) -> bool:
    """`segment_budget_query_due` for the selected search's budgets."""
    if not budget_param_names("search", status.search_mode):
        return False
    return _mirror_query_due(status, status.search_budget, now_s, last_query_s, in_flight)


def _ms(seconds: float) -> str:
    text = f"{seconds * 1e3:.1f}"
    return text[:-2] if text.endswith(".0") else text


def _format_mode_line(label: str, mode: str, budget: tuple[float, ...] | None) -> str:
    """`<label> mode: <mode>` plus budgets and the sim-only mark, when known.

    The budget parameters are seconds; the operator reads milliseconds.
    """
    parts: list[str] = []
    if budget is not None and mode == "nlp" and len(budget) == 3:
        parts += [
            f"wake budget {_ms(budget[0])} ms",
            f"solve {_ms(budget[1])} ms",
            f"≤ {int(budget[2])} solves",
        ]
    elif budget is not None and mode in _SEGMENT_BUDGET_PARAMS and len(budget) == 2:
        parts += [f"first {_ms(budget[0])} ms", f"replan {_ms(budget[1])} ms"]
    if parts and mode in _SIM_ONLY_MODES:
        # Static (no query), but only on a line that already carries budgets:
        # until they are read the line is the mode alone, as before.
        parts.append("sim only")
    return f"{label} mode: {mode}" + (" — " + " · ".join(parts) if parts else "")


def mode_name(value: int) -> str:
    """Name for a wire mode, or the raw value when this build does not know it.

    A number is shown rather than "UNKNOWN" on purpose: an unnamed value means
    the GUI and the controller are from different builds, and the number is
    what identifies which one.
    """
    if 0 <= value < len(MODE_NAMES):
        return MODE_NAMES[value]
    return f"mode:{value}"


def plan_reason_name(value: int) -> str:
    if 0 <= value < len(PLAN_REASON_NAMES):
        return PLAN_REASON_NAMES[value]
    return f"?({value})"


def outcome_name(value: int) -> str:
    if 0 <= value < len(OUTCOME_NAMES):
        return OUTCOME_NAMES[value]
    return f"outcome:{value}"


def hand_phase_name(value: int) -> str:
    if 0 <= value < len(HAND_PHASE_NAMES):
        return HAND_PHASE_NAMES[value]
    return f"phase:{value}"


def reason_name(value: int) -> str:
    if 0 <= value < len(REASON_NAMES):
        return REASON_NAMES[value]
    return f"reason:{value}"


@dataclass
class CatchingStatus:
    """What the last CatchingState said, plus the operator's own pending request."""

    feed: FeedStatus = field(default_factory=lambda: FeedStatus("catching_state"))

    mode: int = 0
    reason: int = 0
    armed: bool = False
    estop_active: bool = False
    fault_latched: bool = False
    armable: bool = False
    law_enabled: bool = False
    tick: int = 0
    #: The controller's `planner.segment.mode`, once read; None = not read yet.
    #: Cached because the value is fixed at the node's first configure.
    segment_mode: str | None = None
    #: The controller's `planner.search.mode` (grid | nlp), read and cached the same way.
    search_mode: str | None = None
    #: Budgets [s] of the selected planners, in `budget_param_names` order; None =
    #: not read (or the mode has none). Reset together with the modes they belong to.
    segment_budget: tuple[float, ...] | None = None
    search_budget: tuple[float, ...] | None = None

    input_valid: bool = False
    input_stale: bool = True
    input_expired: bool = False
    input_n: int = 0
    input_generation: int = 0
    input_snapshot_sequence: int = 0
    input_age_s: float = 0.0
    input_horizon_s: float = 0.0
    input_accept_count: int = 0
    input_reject_counts: tuple[int, ...] = ()
    input_reject_names: tuple[str, ...] = ()

    plan_valid: bool = False
    plan_id: int = 0
    plan_age_s: float = 0.0
    plan_w5: float = 0.0
    plan_w6: float = 0.0
    plan_reason: int = 0
    plan_p_c: tuple[float, float, float] = (0.0, 0.0, 0.0)
    plan_t_c_s: float = 0.0
    plan_gamma_f: float = 0.0

    ref_valid: bool = False
    ref_saturated: bool = False
    track_err_rad: float = 0.0
    clik_ran: bool = False
    clik_converged: bool = False
    clik_status: int = -1
    clik_iterations: int = 0
    clik_solve_us: float = 0.0
    clik_bound_conflict: bool = False
    qp_fail_streak: int = 0

    # S7: the attempt's verdict, the hand sequencer, the fingertips.
    outcome: int = 0
    hand_phase_valid: bool = False
    hand_phase: int = 0
    hand_rho: float = 0.0
    hand_timeout: bool = False
    tip_names: tuple[str, ...] = ()
    tip_force: tuple[float, ...] = ()
    tip_contact: tuple[bool, ...] = ()
    tip_fresh: tuple[bool, ...] = ()
    tip_age_s: tuple[float, ...] = ()

    #: What the operator last asked ``catching.enable`` to be, and whether that
    #: request is still in flight. ``None`` means nothing has been requested in
    #: this session, so there is nothing to compare the observed latch against.
    requested_arm: bool | None = None
    request_pending: bool = False
    #: The tick the request was made against. A request stops being in flight
    #: once a LATER tick has published — the tick has had its say by then,
    #: whether or not it agreed.
    request_tick: int = 0
    last_error: str = ""

    #: S8 progress (L8 §11): attempts seen by THIS panel, by verdict. An
    #: attempt is counted on the tick-published edge into RETREAT, where the
    #: supervisor publishes the attempt's verdict (L7 §4.4) — the same edge the
    #: trial runner reads, so the panel and trial_results.json count alike. The
    #: verdict itself cannot be the edge: it is kept across a re-arm, so two
    #: Missed in a row would read as one. Local to the panel (no message field —
    #: CatchingState is frozen since S5); a panel started mid-run counts from
    #: the first edge it sees, and restarting the panel starts a new count.
    attempt_tally: dict[str, int] = field(default_factory=dict)
    _prev_mode: int | None = None
    _prev_tick: int | None = None

    def update(self, msg, now_s: float) -> None:
        """Adopt one CatchingState. Duck-typed so tests need no ROS message."""
        # A feed that went silent and came back may be another controller
        # process: the cached segment mode is its predecessor's until read again.
        # (The tick check below catches a restart the gap did not show; this
        # one catches a restart whose first tick is ABOVE the last one seen —
        # a controller relaunched in the other mode and switched in later.)
        if self.feed.state(now_s) is FeedState.STALE:
            self.segment_mode = None
            self.search_mode = None
            self.segment_budget = None
            self.search_budget = None
        self.feed.mark(now_s)
        prev_mode = self._prev_mode
        self.mode = int(msg.mode)
        self._prev_mode = self.mode
        self.reason = int(msg.reason)
        self.armed = bool(msg.armed)
        self.estop_active = bool(msg.estop_active)
        self.fault_latched = bool(msg.fault_latched)
        self.armable = bool(msg.armable)
        self.law_enabled = bool(msg.law_enabled)
        new_tick = int(msg.tick)
        # The tick counter restarting means the controller process (or its
        # activation) restarted, so a cached law may belong to the previous
        # process and is read again. The first message has nothing to compare.
        if self._prev_tick is not None and new_tick < self._prev_tick:
            self.segment_mode = None
            self.search_mode = None
            self.segment_budget = None
            self.search_budget = None
        self._prev_tick = new_tick
        self.tick = new_tick

        self.input_valid = bool(msg.input_valid)
        self.input_stale = bool(msg.input_stale)
        self.input_expired = bool(msg.input_expired)
        self.input_n = int(msg.input_n)
        self.input_generation = int(msg.input_generation)
        self.input_snapshot_sequence = int(msg.input_snapshot_sequence)
        self.input_age_s = float(msg.input_age_s)
        self.input_horizon_s = float(msg.input_horizon_s)
        self.input_accept_count = int(msg.input_accept_count)
        self.input_reject_counts = tuple(int(c) for c in msg.input_reject_counts)
        self.input_reject_names = tuple(str(n) for n in msg.input_reject_names)

        self.plan_valid = bool(msg.plan_valid)
        self.plan_id = int(msg.plan_id)
        self.plan_age_s = float(msg.plan_age_s)
        self.plan_w5 = float(msg.plan_w5)
        self.plan_w6 = float(msg.plan_w6)
        self.plan_reason = int(msg.plan_reason)
        self.plan_p_c = tuple(float(v) for v in msg.plan_p_c)
        self.plan_t_c_s = float(msg.plan_t_c_s)
        self.plan_gamma_f = float(msg.plan_gamma_f)

        self.ref_valid = bool(msg.ref_valid)
        self.ref_saturated = bool(msg.ref_saturated)
        self.track_err_rad = float(msg.track_err_rad)
        self.clik_ran = bool(msg.clik_ran)
        self.clik_converged = bool(msg.clik_converged)
        self.clik_status = int(msg.clik_status)
        self.clik_iterations = int(msg.clik_iterations)
        self.clik_solve_us = float(msg.clik_solve_us)
        self.clik_bound_conflict = bool(msg.clik_bound_conflict)
        self.qp_fail_streak = int(msg.qp_fail_streak)

        # S7 fields. Read with defaults so a recording from before S7 (whose
        # values were published but always zero) and a duck-typed test message
        # without them both read as "not computed" rather than failing.
        self.outcome = int(getattr(msg, "outcome", 0))
        self.hand_phase_valid = bool(getattr(msg, "hand_phase_valid", False))
        self.hand_phase = int(getattr(msg, "hand_phase", 0))
        self.hand_rho = float(getattr(msg, "hand_rho", 0.0))
        self.hand_timeout = bool(getattr(msg, "hand_timeout", False))
        self.tip_names = tuple(str(n) for n in getattr(msg, "tip_names", ()))
        self.tip_force = tuple(float(v) for v in getattr(msg, "tip_force", ()))
        self.tip_contact = tuple(bool(v) for v in getattr(msg, "tip_contact", ()))
        self.tip_fresh = tuple(bool(v) for v in getattr(msg, "tip_fresh", ()))
        self.tip_age_s = tuple(float(v) for v in getattr(msg, "tip_age_s", ()))

        if prev_mode is not None and prev_mode != self.mode and mode_name(self.mode) == "RETREAT":
            verdict = outcome_name(self.outcome)
            self.attempt_tally[verdict] = self.attempt_tally.get(verdict, 0) + 1

        # A request stops being in flight once a LATER TICK has published, not
        # once the latch happens to match. The tick is what decides, and it can
        # decide NO — an arm pressed while a fault is latched is refused
        # outright, and waiting for agreement would leave that request "in
        # flight" forever while suppressing the very banner this panel exists
        # to raise.
        if self.request_pending and self.tick > self.request_tick:
            self.request_pending = False

    def note_request(self, value: bool) -> None:
        """Record that the operator asked for a new arming state."""
        self.requested_arm = value
        self.request_pending = True
        self.request_tick = self.tick
        self.last_error = ""

    def arming_disagrees(self) -> bool:
        """The controller is not where the operator asked it to be, and is not
        still on its way there."""
        if self.requested_arm is None or self.request_pending:
            return False
        return self.armed != self.requested_arm

    def alarm(self) -> bool:
        """Worth colouring. E-STOP and a latched fault are not modes, so mode
        alone would miss both."""
        return self.estop_active or self.fault_latched or mode_name(self.mode) in _ALARM_MODES

    def tally_summary(self) -> str:
        """``attempts N: CAPTURED a · MISSED b …`` in verdict order; empty before
        the first attempt."""
        total = sum(self.attempt_tally.values())
        if total == 0:
            return ""
        order = [n for n in OUTCOME_NAMES if n in self.attempt_tally]
        order += sorted(n for n in self.attempt_tally if n not in OUTCOME_NAMES)
        parts = " · ".join(f"{name} {self.attempt_tally[name]}" for name in order)
        return f"attempts {total}: {parts}"

    def reject_summary(self) -> str:
        """Non-zero reject counters only, named. Empty string when the lane has
        refused nothing — a row of zeroes is noise that hides the one that is
        not zero. Buckets in ``_NOT_A_REJECT`` are never listed."""
        pairs = []
        for i, count in enumerate(self.input_reject_counts):
            if count <= 0:
                continue
            name = self.input_reject_names[i] if i < len(self.input_reject_names) else f"#{i}"
            if name in _NOT_A_REJECT:
                continue
            # `name=count`, the same shape DrainControllerLogs uses for its
            # drop counters. An `x` separator ran once and read as part of the
            # name on screen ("shapex90").
            pairs.append(f"{name}={count}")
        return ", ".join(pairs)

    def lines(self, now_s: float) -> list[str]:
        state = self.feed.state(now_s)
        if state is FeedState.NEVER:
            return [
                f"catching_state: never received (is /{CATCHING_CONFIG_KEY} active?)",
                # A controller PARKED at configure (A-S5-1 / A-S5-12, or the
                # planner and the oracle both enabled) refuses activation and
                # so never publishes: the robot is up, catching is not. Say
                # where the reason is rather than leave "never received" to
                # read as a dead topic.
                "  parked? a controller parked at configure refuses to activate — "
                "its configure log names the values (DISABLED: ...)",
                f"arm: requested {self._requested_text()}",
            ]

        feed_note = "" if state is FeedState.LIVE else "  [STALE FEED]"
        out = [
            f"mode: {mode_name(self.mode)}  reason: {reason_name(self.reason)}"
            f"  last attempt: {outcome_name(self.outcome)}  tick {self.tick}{feed_note}",
            self._segment_mode_line(),
            self._search_mode_line(),
            self._arm_line(),
            self._input_line(),
            self._plan_line(),
            *self._plan_hint_lines(),
            self._law_line(),
            self._hand_line(),
            self._tips_line(),
        ]
        tally = self.tally_summary()
        if tally:
            out.append(f"this panel: {tally}")
        rejects = self.reject_summary()
        if rejects:
            out.append(f"input rejects: {rejects}")
        if self.last_error:
            out.append(f"! {self.last_error}")
        return out

    # ── line builders ──────────────────────────────────────────────────────

    def _requested_text(self) -> str:
        if self.requested_arm is None:
            return PLACEHOLDER
        return "ARMED" if self.requested_arm else "DISARMED"

    def _segment_mode_line(self) -> str:
        if self.segment_mode is None:
            # Not read yet, or never declared: the controller mirrors the key
            # only once it has configured (a parked one answers with nothing).
            return f"segment mode: unknown ({CATCHING_SEGMENT_MODE_PARAM} not read)"
        return _format_mode_line("segment", self.segment_mode, self.segment_budget)

    def _search_mode_line(self) -> str:
        if self.search_mode is None:
            return f"search mode: unknown ({CATCHING_SEARCH_MODE_PARAM} not read)"
        return _format_mode_line("search", self.search_mode, self.search_budget)

    def _arm_line(self) -> str:
        observed = "ARMED" if self.armed else "DISARMED"
        parts = [f"arm: {observed} (observed)"]
        if self.requested_arm is not None:
            suffix = " …" if self.request_pending else ""
            parts.append(f"requested {self._requested_text()}{suffix}")
        if self.arming_disagrees():
            # Name the two things that lower the latch behind the operator's
            # back, so the next action is obvious rather than "try again".
            cause = (
                "E-STOP"
                if self.estop_active
                else ("a latched FAULT" if self.fault_latched else "the controller")
            )
            parts.append(f"<- {cause} lowered it; no automatic resume")
        if not self.armable:
            parts.append("profile NOT armable")
        if not self.law_enabled:
            parts.append("law not wired (arm holds)")
        return "  |  ".join(parts)

    def _input_line(self) -> str:
        if not self.input_valid:
            return (
                f"input: no usable prediction (stale={int(self.input_stale)} "
                f"expired={int(self.input_expired)}), accepted {self.input_accept_count}"
            )
        verdict = []
        if self.input_stale:
            verdict.append("STALE")
        if self.input_expired:
            verdict.append("EXPIRED")
        tag = (" " + "/".join(verdict)) if verdict else ""
        return (
            f"input:{tag} n={self.input_n} gen={self.input_generation} "
            f"seq={self.input_snapshot_sequence} age={self.input_age_s * 1e3:.1f} ms "
            f"horizon={self.input_horizon_s:.3f} s"
        )

    def _plan_hint_lines(self) -> list[str]:
        """Why "no plan" may name nothing, for planners that withhold the whole plan."""
        if not self.plan_valid and self.segment_mode in _PLAN_WITH_FIRST_SEGMENT_MODES:
            return [PLAN_WITH_FIRST_SEGMENT_HINT]
        return []

    def _plan_line(self) -> str:
        if not self.plan_valid:
            # The planner's reason for "no plan" rides the same message when it
            # has spoken this activation (plan_id > 0); before that there is
            # nothing to name, and NONE would read as "no problem".
            if self.plan_id > 0:
                return (
                    f"plan: none — {plan_reason_name(self.plan_reason)} "
                    f"(planner #{self.plan_id}, {self.plan_age_s * 1e3:.0f} ms ago)"
                )
            return "plan: none"
        px, py, pz = self.plan_p_c
        return (
            f"plan #{self.plan_id}: p_c=({px:+.3f}, {py:+.3f}, {pz:+.3f}) m  "
            f"t_c={self.plan_t_c_s:+.3f} s  gamma_f={self.plan_gamma_f:.2f}  "
            f"w5={self.plan_w5:.3f} w6={self.plan_w6:.4f}  age={self.plan_age_s * 1e3:.0f} ms"
        )

    def _law_line(self) -> str:
        if not self.clik_ran:
            # An un-run solve is NOT status 0: 0 is ProxQP's SOLVED, and
            # printing it here would report a solve that never happened.
            err = PLACEHOLDER if not self.ref_valid else f"{self.track_err_rad:.4f}"
            return f"law: not solved this tick  |  track err {err} rad"
        verdict = "SOLVED" if self.clik_converged else f"FAILED (status {self.clik_status})"
        parts = [
            f"law: {verdict} {self.clik_iterations} it, {self.clik_solve_us:.0f} us",
            f"track err {self.track_err_rad:.4f} rad",
        ]
        if self.ref_saturated:
            parts.append("reference SATURATED")
        if self.clik_bound_conflict:
            parts.append("BOUND CONFLICT")
        if self.qp_fail_streak > 0:
            # Trials in a row the solver ended (#537 S9b), not failed solves:
            # it stays up across good solves until a verdict or a fault reset.
            parts.append(f"QP fail streak {self.qp_fail_streak} trial(s)")
        return "  |  ".join(parts)

    def _hand_line(self) -> str:
        if not self.hand_phase_valid:
            # The latch commands the hand: not armed, E-STOP, the step rig, or a
            # configuration that cannot run a trial.
            return "hand: on the latch (sequencer inactive)"
        parts = [f"hand: {hand_phase_name(self.hand_phase)}", f"rho {self.hand_rho:.2f}"]
        if self.hand_timeout:
            parts.append("CLOSE TIMEOUT (rho < eta)")
        return "  |  ".join(parts)

    def _tips_line(self) -> str:
        n = len(self.tip_age_s)
        if n == 0:
            return "tips: no fingertip lane"
        cells = []
        for i in range(n):
            name = self.tip_names[i] if i < len(self.tip_names) else f"#{i}"
            age = self.tip_age_s[i]
            if age < 0:
                cells.append(f"{name}: never")
                continue
            fresh = i < len(self.tip_fresh) and self.tip_fresh[i]
            force = self.tip_force[i] if i < len(self.tip_force) else 0.0
            contact = i < len(self.tip_contact) and self.tip_contact[i]
            tag = "CONTACT " if contact else ""
            stale = "" if fresh else " STALE"
            cells.append(f"{name}: {tag}{force:.2f} N{stale} ({age * 1e3:.0f} ms)")
        return "tips: " + "  ".join(cells)
