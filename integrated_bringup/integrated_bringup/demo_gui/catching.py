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

# Modes the operator should be able to spot without reading the word.
_ALARM_MODES = frozenset({"ABORT_SAFE", "FAULT"})


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

    def update(self, msg, now_s: float) -> None:
        """Adopt one CatchingState. Duck-typed so tests need no ROS message."""
        self.feed.mark(now_s)
        self.mode = int(msg.mode)
        self.reason = int(msg.reason)
        self.armed = bool(msg.armed)
        self.estop_active = bool(msg.estop_active)
        self.fault_latched = bool(msg.fault_latched)
        self.armable = bool(msg.armable)
        self.law_enabled = bool(msg.law_enabled)
        self.tick = int(msg.tick)

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

    def reject_summary(self) -> str:
        """Non-zero reject counters only, named. Empty string when the lane has
        refused nothing — a row of zeroes is noise that hides the one that is
        not zero."""
        pairs = []
        for i, count in enumerate(self.input_reject_counts):
            if count <= 0:
                continue
            name = self.input_reject_names[i] if i < len(self.input_reject_names) else f"#{i}"
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
            f"  tick {self.tick}{feed_note}",
            self._arm_line(),
            self._input_line(),
            self._plan_line(),
            self._law_line(),
        ]
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
            parts.append(f"QP fail streak {self.qp_fail_streak}")
        return "  |  ".join(parts)
