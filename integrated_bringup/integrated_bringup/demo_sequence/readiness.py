"""What the ROS graph shows, and whether a run may start — rclpy-free.

``Observation`` is one look at the stack, taken by the ROS side (``ros_io``)
after it has handled every callback that was ready. Every judgement on it is a
pure function here, so the table of the operator document is the table pytest
pins:

- ``assess`` — before RUNNING: wait (with what is missing), refuse (exit 3),
  or ready,
- ``runtime_abort`` — every tick of RUNNING, whatever the sequence is doing.

A latched value (active controller, E-STOP) is evidence only while its
publisher is alive: a CM that died leaves the last sample in our cache, so the
publisher count is part of every check that reads one.
"""

from __future__ import annotations

from dataclasses import dataclass, field

CONTROLLER = "hand_joint_controller"
TRACKING = "tracking"

# A joint_states gap above this ends a run (and restarts the readiness count).
STALE_S = 1.0
# The stream must have run without such a gap this long before a run starts.
STEADY_S = 2.0

WAIT = "wait"
REFUSE = "refuse"
READY = "ready"


@dataclass(frozen=True)
class Observation:
    now: float
    cm_nodes: int = 0  # nodes named integrated_rt_controller
    param_nodes: int = 0  # /hand_joint_controller/hand_joint_controller
    active_pubs: int = 0  # publishers of /rtc_cm/active_controller_name
    estop_pubs: int = 0  # publishers of /system/estop_status
    active: str | None = None  # last latched sample (None = never)
    estop: bool | None = None
    # joint_states of this hand: None = never received
    joint_age: float | None = None
    steady_s: float = 0.0  # how long the stream has run without a STALE_S gap
    roster_ok: bool | None = None  # names of the last sample == the profile's (None = no sample)
    positions: tuple[float, ...] = ()  # in the profile's joint order [rad]
    link_pubs: int = 0  # publishers of this hand's link_status
    link: bool | None = None
    goal_pubs: int = 0  # graph publishers of joint_goal, this CLI's included
    goal_matched: int = 0  # subscribers matched to this CLI's publisher
    mode: str | None = None  # controller `mode` as last read
    cap: float | None = None  # max_traj_velocity [rad/s] as last read
    other_hand: bool = False  # the other hand's joint_states is on this domain


@dataclass(frozen=True)
class Verdict:
    kind: str
    reason: str = ""
    missing: tuple[str, ...] = field(default=())


def _refusal(obs: Observation, cm_seen: bool) -> str:
    """Why waiting cannot help (empty = it can)."""
    if obs.estop_pubs > 0 and obs.estop:
        return "E-STOP is latched — clear it (or restart the bring-up) first"
    if obs.other_hand:
        return (
            "the other hand's stack is on this ROS_DOMAIN_ID — give each NUC its own "
            "domain (operator doc)"
        )
    for count, what in (
        (obs.cm_nodes, "controller managers (integrated_rt_controller)"),
        (obs.param_nodes, "hand_joint_controller nodes"),
        (obs.active_pubs, "publishers of /rtc_cm/active_controller_name"),
        (obs.estop_pubs, "publishers of /system/estop_status"),
    ):
        if count > 1:
            return f"{count} {what} on this domain — exactly one stack may be visible"
    if cm_seen and obs.cm_nodes == 0:
        return (
            "the controller manager disappeared while starting (init timeout?) — check the "
            "hand power, CAN and the driver, then restart the bring-up"
        )
    if obs.active_pubs > 0 and obs.active is not None and obs.active != CONTROLLER:
        return f"the active controller is '{obs.active}', not {CONTROLLER} (no switch is made)"
    if obs.mode is not None and obs.mode != TRACKING:
        return f"{CONTROLLER} mode is '{obs.mode}', not {TRACKING} — goals would be refused"
    if obs.goal_pubs > 1:
        return f"another publisher on the hand's joint_goal ({obs.goal_pubs - 1}) — stop it first"
    if obs.roster_ok is False:
        return "the hand's joint_states names do not match this profile (wrong --robot?)"
    return ""


def _missing(obs: Observation) -> tuple[str, ...]:
    out = []
    if obs.cm_nodes == 0:
        out.append("controller manager node")
    if obs.param_nodes == 0:
        out.append(f"{CONTROLLER} node")
    if obs.active_pubs == 0 or obs.active is None:
        out.append("active controller")
    if obs.estop_pubs == 0 or obs.estop is None:
        out.append("E-STOP status")
    if obs.joint_age is None or obs.joint_age > STALE_S:
        out.append("joint_states")
    elif obs.steady_s < STEADY_S:
        out.append(f"joint_states steady {STEADY_S:.0f} s")
    if obs.mode is None or obs.cap is None:
        # Topics cross RMW vendors here but services do not answer across them:
        # a visible node whose parameters never come is that, most of the time.
        out.append(
            f"{CONTROLLER} parameters"
            + (
                " (its node is visible but does not answer — the same RMW_IMPLEMENTATION "
                "on both sides?)"
                if obs.param_nodes == 1
                else ""
            )
        )
    if obs.goal_matched == 0:
        out.append("joint_goal subscriber")
    if obs.link_pubs > 0 and obs.link is not True:
        out.append("hand link up")
    return tuple(out)


def assess(obs: Observation, cm_seen: bool) -> Verdict:
    """``cm_seen``: a CM node was seen earlier in this wait."""
    if reason := _refusal(obs, cm_seen):
        return Verdict(REFUSE, reason)
    if missing := _missing(obs):
        return Verdict(WAIT, "waiting for: " + ", ".join(missing), missing)
    return Verdict(READY)


def runtime_abort(obs: Observation, link_required: bool) -> str:
    """Why the run must stop now ("" = it may go on). Checked every tick."""
    if obs.joint_age is None or obs.joint_age > STALE_S:
        age = "never" if obs.joint_age is None else f"{obs.joint_age:.2f} s ago"
        return f"no hand joint_states for over {STALE_S:.1f} s (last {age})"
    if obs.active_pubs == 0:
        return "the controller manager is gone (no active-controller publisher)"
    if obs.estop_pubs == 0:
        return "the E-STOP status has no publisher any more — no E-STOP evidence"
    if obs.estop_pubs > 0 and obs.estop:
        return "E-STOP latched"
    if link_required and (obs.link_pubs == 0 or obs.link is not True):
        return "the hand link is down (power cut or CAN) — restart the bring-up after recovery"
    if obs.goal_matched == 0:
        return f"{CONTROLLER} no longer subscribes to joint_goal"
    reason = _refusal(obs, cm_seen=True)
    if reason:
        return reason
    if obs.cm_nodes == 0:
        return "the controller manager node is gone"
    return ""
