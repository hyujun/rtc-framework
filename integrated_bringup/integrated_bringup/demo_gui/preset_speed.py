"""Hand speed of a preset send for demo_controller_gui — Tk- and rclpy-free.

A preset stores a ``grasp_time``; the GUI turns it into the hand trajectory
speed of the controller the goal goes to. Three things decide whether that
works, and they live here so pytest can pin them:

- the parameter is not spelled the same on every controller
  (``hand_speed_params``),
- every controller refuses a speed above its read-only cap, so the speed is
  lowered to the cap rather than sent and refused (``plan_hand_speed``),
- a quintic's peak velocity is 15/8 of its mean, so a move at the cap takes
  longer than ``grasp_time`` (``HandSpeedPlan.min_duration``).
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

# (speed parameter, its read-only cap) as the hand-carrying demo controllers
# declare them, and as hand_joint_controller does (one device, no prefix).
DEMO_HAND_SPEED_PARAMS: tuple[str, str] = ("hand_trajectory_speed", "hand_max_traj_velocity")
HAND_JOINT_SPEED_PARAMS: tuple[str, str] = ("trajectory_speed", "max_traj_velocity")
# demo_dualarm_controller (rows built from its YAML, same two parameters on
# every robot).
DUALARM_HAND_SPEED_PARAMS: tuple[str, str] = (
    "trajectory.hand_speed",
    "trajectory.hand_speed_max",
)
# Pairs recognised by the speed parameter's name in a controller's gain rows.
# HAND_JOINT_SPEED_PARAMS is not one of them: `trajectory_speed` is the ARM's
# speed on the task-space controllers, so that pair goes by controller key.
_PAIRS_BY_DECLARED_NAME = (DEMO_HAND_SPEED_PARAMS, DUALARM_HAND_SPEED_PARAMS)

# Peak / mean velocity of a rest-to-rest quintic
# (kQuinticPeakVelocityFactor in the controllers).
QUINTIC_PEAK_FACTOR = 15.0 / 8.0

# Below these a grasp_time is not a usable speed request (a hand already at the
# preset, a zero time): the speed falls back to FALLBACK_SPEED.
MIN_DISTANCE = 0.001  # rad
MIN_GRASP_TIME = 0.01  # s
FALLBACK_SPEED = 1.0  # rad/s


def hand_speed_params(
    ctrl: str,
    dispatch: Mapping[str, Mapping[str, tuple]],
    hand_joint_owners: Sequence[str] | frozenset[str],
) -> tuple[str, str] | None:
    """(speed parameter, cap parameter) of ``ctrl``'s hand trajectory, or None.

    ``dispatch`` is ``config.GAIN_PARAM_DISPATCH``: a controller whose rows name
    a known hand speed parameter declares that pair. None for a controller that
    declares none — its preset is sent without a speed set.
    """
    if ctrl in hand_joint_owners:
        return HAND_JOINT_SPEED_PARAMS
    declared = {param for param, _ in dispatch.get(ctrl, {}).values()}
    for pair in _PAIRS_BY_DECLARED_NAME:
        if pair[0] in declared:
            return pair
    return None


@dataclass(frozen=True)
class HandSpeedPlan:
    """What a preset send sets and what it may promise.

    ``speed`` is the value to set [rad/s]. ``capped`` is True when
    ``grasp_time`` asked for more than the cap. ``min_duration`` [s] is the
    shortest the move can take: a controller with per-joint device ratings
    below the cap stretches it further, so it is a lower bound.
    """

    speed: float
    capped: bool
    min_duration: float


def plan_hand_speed(max_dist: float, grasp_time: float, cap: float | None) -> HandSpeedPlan:
    """Speed for a move of ``max_dist`` [rad] meant to take ``grasp_time`` [s].

    ``cap`` is the controller's read-only maximum [rad/s]; None (not read, not
    finite, not positive) leaves the speed uncapped and the controller the
    judge.
    """
    usable = max_dist > MIN_DISTANCE and grasp_time > MIN_GRASP_TIME
    wanted = max_dist / grasp_time if usable else FALLBACK_SPEED
    if cap is None or not math.isfinite(cap) or cap <= 0.0:
        return HandSpeedPlan(wanted, False, max_dist / wanted)
    speed = min(wanted, cap)
    min_duration = max(max_dist / speed, QUINTIC_PEAK_FACTOR * max_dist / cap)
    return HandSpeedPlan(speed, usable and wanted > cap, min_duration)


def describe_plan(plan: HandSpeedPlan, grasp_time: float, max_dist: float) -> str:
    """One log line: the speed set and how long the move takes at least."""
    text = (
        f"speed={plan.speed:.4f} rad/s (grasp_time={grasp_time:.2f}s, "
        f"max_dist={max_dist:.4f} rad), takes >= {plan.min_duration:.2f}s"
    )
    if plan.capped:
        text += " — grasp_time asks for more than the controller's cap, speed lowered to it"
    elif plan.min_duration > grasp_time + 1e-6 and max_dist > MIN_DISTANCE:
        text += " — longer than grasp_time: the quintic's peak velocity is held under the cap"
    return text
