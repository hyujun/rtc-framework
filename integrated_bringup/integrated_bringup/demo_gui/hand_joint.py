"""Hand Joint panel logic for demo_controller_gui — Tk- and rclpy-free.

``hand_joint_controller`` (profile ``p2``) declares a ``mode`` parameter and
accepts a set at any time, active or not:

- ``tracking``: joint goals are followed by a quintic trajectory (the default),
- ``damping``: the reference is the measured position every tick with no
  feedforward, so only the device's own kd acts,
- ``damping_feedforward``: the same reference with the gravity feedforward lane
  left on, to test the feedforward.

While the mode is not ``tracking`` the controller refuses every ``joint_goal``.
This module holds what the Control tab's Hand Joint panel decides: who owns the
parameter, how a ``get_parameters`` answer reads, and when the GUI should not
send a hand target at all. The controller's refusal stays the authority; the
send guard only spares the operator a goal that is counted as a reject.

The other runtime parameters (``gravity_ff.*``, ``trajectory_speed``) need no
panel of their own: ``hand_joint_gain_rows`` registers them with the Gains panel.
"""

from __future__ import annotations

from collections.abc import Sequence

from rcl_interfaces.msg import ParameterType

from .dualarm import GainRow
from .param_readout import ReadoutSpec

HAND_JOINT_MODE_PARAM = "mode"

# The controller's whitelist (kHandJointModeNames in hand_joint_controller.hpp),
# in the C++ order, with the panel's readout text for each.
_MODE_TEXT = {
    "tracking": "tracking: joint goals are followed",
    "damping": "damping: reference = measured position, no feedforward (device kd only)",
    "damping_feedforward": "damping_feedforward: reference = measured position, gravity ff on",
}
HAND_JOINT_MODES: tuple[str, ...] = tuple(_MODE_TEXT)
_TRACKING = HAND_JOINT_MODES[0]

# Controllers that declare ``mode``. The only definition here.
HAND_JOINT_OWNERS: frozenset[str] = frozenset({"hand_joint_controller"})


def parse_mode(values: Sequence) -> str | None:
    """Turn a ``get_parameters(["mode"])`` answer into a mode name.

    ``values`` are ``ParameterValue`` (duck-typed: ``type``, ``string_value``).
    None for anything else — PARAMETER_NOT_SET before configure, a wrong type, a
    name outside the whitelist — so the panel shows unknown, never a guess.
    """
    if len(values) != 1:
        return None
    v = values[0]
    if v.type != ParameterType.PARAMETER_STRING or v.string_value not in HAND_JOINT_MODES:
        return None
    return v.string_value


HAND_JOINT_READOUT = ReadoutSpec(
    params=(HAND_JOINT_MODE_PARAM,),
    owners=HAND_JOINT_OWNERS,
    parse=parse_mode,
    unknown=None,
    label=HAND_JOINT_MODE_PARAM,
)


def format_mode_readout(mode: str | None) -> str:
    """One line for the panel's readout of the controller's current mode."""
    if mode is None:
        return "current: unknown"
    return f"current: {_MODE_TEXT[mode]}"


def goal_block_reason(mode: str | None) -> str:
    """Why a hand target must not be sent now; "" when it may be.

    Fail-open: an unknown mode (not read yet, not an owner, unreadable) does not
    block — the controller refuses what it must, and a guard that closed on
    unknown would keep Send off on every controller that has no mode.
    """
    if mode is None or mode == _TRACKING:
        return ""
    return f"mode is {mode}: joint goals are refused"


GAIN_GROUP_FF = "Gravity feedforward"
GAIN_GROUP_TRAJ = "Trajectory"
# Gains-panel box layout: one row, the two groups side by side.
HAND_JOINT_GAIN_LAYOUT: list[list[str]] = [[GAIN_GROUP_FF, GAIN_GROUP_TRAJ]]


def hand_joint_gain_rows() -> tuple[GainRow, ...]:
    """The controller's numeric/bool runtime parameters as Gains-panel rows.

    Defaults are config/p2/controllers/hand_joint_controller.yaml's; Load Gain
    replaces them with the live values. ``max_traj_velocity`` is read-only.
    """
    return (
        GainRow("gravity_ff enabled", "gravity_ff.enabled", 1.0, False, GAIN_GROUP_FF, True),
        GainRow("gravity_ff gain [0-2]", "gravity_ff.gain", 1.0, False, GAIN_GROUP_FF),
        GainRow("trajectory_speed [rad/s]", "trajectory_speed", 1.0, False, GAIN_GROUP_TRAJ),
        GainRow("max_traj_velocity [rad/s]", "max_traj_velocity", 2.0, True, GAIN_GROUP_TRAJ),
    )
