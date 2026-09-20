"""Hand step panel state for demo_controller_gui (dynamic_catching §13 S4).

Two buttons — *Step → preshape* and *Step → closed* — plus a live rho readout,
for the S4.0 catching controller's diagnostic mode. It exists so the S4.1 hand
postures can be LOOKED AT in sim before S4.2 measures anything with them: a
posture that does not cage the ball produces a perfectly clean T_close
distribution for a grasp that would drop it.

**The profile comes from the controller, never from this file.** The panel is
handed the numbers the controller mirrored into its read-only parameters
(``hand.q_pre`` / ``hand.q_close`` / ``hand.caging_mask`` / ``hand.eta_close``).
Hard-coding a posture here would make the GUI show one pose while the run used
another, and the screenshot taken to confirm the pose would then confirm the
wrong thing.

**rho is imported, not reimplemented.** ``rtc_tools.analysis.hand_close.rho`` is
the same function ``analyze_hand_close`` uses offline, so the number on screen
and the number in the report cannot drift apart. A copy here is exactly the
drift the import prevents; this module holds no formula of its own.

Pure Python — no Tk, no rclpy — so it is unit-testable without a display or a
ROS graph, following ``demo_gui.ball_launch`` / ``demo_gui.pull``.

Public surface (imported by app.py):
- HandStepProfile, profile_from_parameters
- HandStepPanel, StepPose
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from rtc_tools.analysis.hand_close import HandProfile, rho

PLACEHOLDER = "--"

# The controller's config_key, which is also its ROS namespace and the stem of
# its shipped YAML (rt_controller_node_params.cpp builds "/<config_key>").
CATCHING_CONFIG_KEY = "demo_catching_controller"

# Read-only parameters the catching controller declares in on_configure.
PROFILE_PARAMETERS = (
    "hand.q_open",
    "hand.q_pre",
    "hand.q_close",
    "hand.caging_mask",
    "hand.eta_close",
    "hand.rho_eps",
    "diagnostic.hand_step",
)


class StepPose(Enum):
    OPEN = "open"
    PRESHAPE = "preshape"
    CLOSED = "closed"


@dataclass(frozen=True)
class HandStepProfile:
    """What the controller loaded, as the panel needs it."""

    q_open: list[float]
    q_pre: list[float]
    q_close: list[float]
    caging_mask: list[bool]
    eta_close: float
    rho_eps: float
    hand_step_enabled: bool

    def target(self, pose: StepPose) -> list[float]:
        if pose is StepPose.OPEN:
            return list(self.q_open)
        if pose is StepPose.PRESHAPE:
            return list(self.q_pre)
        return list(self.q_close)

    def as_hand_profile(self, joint_names: list[str]) -> HandProfile:
        return HandProfile(
            joint_names=list(joint_names),
            q_pre=list(self.q_pre),
            q_close=list(self.q_close),
            caging_mask=list(self.caging_mask),
            eta_close=self.eta_close,
            rho_eps=self.rho_eps,
        )


def profile_from_parameters(values: dict) -> HandStepProfile:
    """Build the panel profile from a ``get_parameters`` result.

    Raises ValueError naming the missing or malformed key. A partially-read
    profile is refused rather than padded: a q_close short by one joint would
    silently command that joint to 0 rad, which on most hands is "fully open".
    """
    missing = [name for name in PROFILE_PARAMETERS if name not in values]
    if missing:
        raise ValueError(
            f"the controller did not report {missing}. Is a catching controller "
            "configured? (It is sim-only and skipped on profiles that ship no YAML.)"
        )
    q_open = [float(v) for v in values["hand.q_open"]]
    q_pre = [float(v) for v in values["hand.q_pre"]]
    q_close = [float(v) for v in values["hand.q_close"]]
    caging = [bool(v) for v in values["hand.caging_mask"]]
    n = len(q_pre)
    if not n:
        raise ValueError("the controller reports an empty hand profile")
    for name, seq in (
        ("hand.q_open", q_open),
        ("hand.q_close", q_close),
        ("hand.caging_mask", caging),
    ):
        if len(seq) != n:
            raise ValueError(f"{name} has {len(seq)} entries, hand.q_pre has {n}")
    return HandStepProfile(
        q_open=q_open,
        q_pre=q_pre,
        q_close=q_close,
        caging_mask=caging,
        eta_close=float(values["hand.eta_close"]),
        rho_eps=float(values["hand.rho_eps"]),
        hand_step_enabled=bool(values["diagnostic.hand_step"]),
    )


@dataclass
class HandStepPanel:
    """Panel state: the loaded profile, the last step sent, and the live rho."""

    profile: HandStepProfile | None = None
    joint_names: list[str] | None = None
    last_sent: StepPose | None = None
    last_error: str = ""
    rho_value: float = math.nan

    def load(self, values: dict, joint_names: list[str]) -> bool:
        """Adopt a controller profile. Returns False and records why on refusal."""
        try:
            profile = profile_from_parameters(values)
        except ValueError as exc:
            self.profile = None
            self.last_error = str(exc)
            return False
        if len(joint_names) != len(profile.q_pre):
            self.profile = None
            self.last_error = (
                f"the hand device reports {len(joint_names)} joints but the profile has "
                f"{len(profile.q_pre)}"
            )
            return False
        self.profile = profile
        self.joint_names = list(joint_names)
        self.last_error = ""
        return True

    def step_target(self, pose: StepPose) -> list[float]:
        """The joint target for one button press.

        Refuses when the controller has the diagnostic off, because the
        controller refuses it too — answering on the button is the difference
        between "nothing happened" and "nothing happened, here is why".
        """
        if self.profile is None:
            raise ValueError(self.last_error or "no hand profile loaded")
        if not self.profile.hand_step_enabled:
            raise ValueError(
                "the controller has diagnostic.hand_step = false and will refuse this "
                "step. Enable it in the controller YAML."
            )
        self.last_sent = pose
        return self.profile.target(pose)

    def update_rho(self, measured: list[float]) -> float:
        """Recompute the live rho from a hand joint-state message."""
        if self.profile is None or self.joint_names is None:
            self.rho_value = math.nan
            return self.rho_value
        if len(measured) < len(self.profile.q_pre):
            # A short state message would make rho read the wrong joints;
            # report "unknown" rather than a number computed from padding.
            self.rho_value = math.nan
            return self.rho_value
        self.rho_value = rho(list(measured), self.profile.as_hand_profile(self.joint_names))
        return self.rho_value

    def lines(self) -> list[str]:
        if self.profile is None:
            return [f"hand profile: {self.last_error or 'not loaded'}"]
        caged = sum(1 for on in self.profile.caging_mask if on)
        sent = self.last_sent.value if self.last_sent else PLACEHOLDER
        if math.isnan(self.rho_value):
            rho_text = PLACEHOLDER
        else:
            verdict = "caged" if self.rho_value >= self.profile.eta_close else "closing"
            rho_text = f"{self.rho_value:+.3f} ({verdict}, eta {self.profile.eta_close:.2f})"
        step_state = "enabled" if self.profile.hand_step_enabled else "DISABLED in YAML"
        return [
            f"hand profile: {len(self.profile.q_pre)} joints, {caged} caging, step {step_state}",
            f"last step: {sent}",
            f"rho: {rho_text}",
        ]
