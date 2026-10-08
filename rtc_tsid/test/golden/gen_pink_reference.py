#!/usr/bin/env python3
"""Record the pink reference of the two-frame CLIK (clik_pink_reference.inc).

pink (https://github.com/stephane-caron/pink) is an independent, published
implementation of the same problem ClikReferenceGenerator's multi-frame
Compute() solves: a weighted least-squares differential IK with a frame task in
the world, a frame task relative to another frame, a posture task and
configuration / velocity limits. This script solves a set of fixed problems
with it and writes the inputs and pink's joint velocities as a C++ fragment;
test_clik_pink_reference.cpp feeds the same inputs to the CLIK and compares.

The reference is RECORDED, not computed at test time: pink is a Python package
the build does not depend on, and a test that silently skips where it is
missing would not be a gate. Re-run this script only to change the problem set
or to move to another pink version, and commit the regenerated fragment with
the reason.

The inputs are seeded and come out bit for bit on a re-run; pink's velocities
come out to round-off only (a few 1e-16 on a handful of values, measured) -
the QP solve is not bitwise deterministic. A regenerated fragment that differs
there is not a change.

How the two formulations map (dt = DT, v = dq / dt):

    pink                               CLIK
    1/2 |cost (J dq + gain e)|^2       weight |J v - K e|^2
      cost                               weight = cost^2
      gain                               K = gain / dt
    damping                            damping_sq
    ConfigurationLimit(gain=1)         the one-step position box
    VelocityLimit                      the velocity box

pink writes its frame tasks in the frame's own axes and the CLIK in the base
frame's; the two differ by a rotation of the rows, which a cost that is the
same on all six rows does not see.

One real difference remains. pink's task Jacobian is the exact derivative of
its error, -Jlog6(T_target_frame) J; the CLIK's velocity law is first order
(J alone, "U1" in se3_error.hpp). They agree to first order in the pose
error, so the fragment holds two tiers:

    tier 1  pink as published, pose errors of 1 mm / 1 mrad
    tier 2  pink with the Jlog6 factor removed (the two task subclasses
            below), pose errors of 0.2 m / 0.5 rad - the same problem as
            the CLIK's, at an error size where Jlog6 would matter

Run, in an environment of its own (pin-pink needs PyPI's `pin`, which must not
land in the workspace's interpreter):

    uv venv <dir>
    uv pip install --python <dir>/bin/python pin-pink==4.4.0 "qpsolvers[proxqp]"
    <dir>/bin/python rtc_tsid/test/golden/gen_pink_reference.py
"""

from __future__ import annotations

import importlib.metadata
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pink
import pinocchio as pin
from pink.limits import ConfigurationLimit, VelocityLimit
from pink.tasks import FrameTask, PostureTask, RelativeFrameTask

REPO = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent / "clik_pink_reference.inc"

DT = 0.01
DAMPING = 1e-6
WORLD_COST, WORLD_GAIN = 1.0, 0.12  # weight 1.0,  K = 12 1/s
REL_COST, REL_GAIN = 0.6, 0.08  # weight 0.36, K = 8 1/s
POSTURE_COST, POSTURE_GAIN = 0.5, 0.02  # weight 0.25, K = 2 1/s
MAX_NV = 17
CASES_PER_TIER = 6
ACTIVE_TOL = 1e-9  # on dq: a limit row is active when it holds with equality


@dataclass(frozen=True)
class Problem:
    """A model and the two frame tasks solved on it."""

    name: str
    urdf: Path
    world_frame: str  # frame of the world task
    rel_frame: str  # frame of the relative task ...
    rel_root: str  # ... and the frame it is relative to


PROBLEMS = (
    Problem(
        "arm9",
        REPO / "robot_descriptions/robots/panda/urdf/panda.urdf",
        "panda_hand",
        "panda_link5",
        "panda_link1",
    ),
    Problem(
        "tree17",
        REPO / "rtc_tsid/test/urdf/tree_17dof.urdf",
        "tip_b",
        "tip_a",
        "torso",
    ),
)


class FirstOrderFrameTask(FrameTask):
    """FrameTask without the Jlog6 factor of its Jacobian."""

    def compute_jacobian(self, configuration: pink.Configuration) -> np.ndarray:
        """Return -J, the body Jacobian of the frame."""
        return -configuration.get_frame_jacobian(self.frame)


class FirstOrderRelativeFrameTask(RelativeFrameTask):
    """RelativeFrameTask without the Jlog6 factor of its Jacobian."""

    def compute_jacobian(self, configuration: pink.Configuration) -> np.ndarray:
        """Return the body Jacobian of the frame relative to the root."""
        transform_frame_to_root = configuration.get_transform(self.frame, self.root)
        return configuration.get_frame_jacobian(
            self.frame
        ) - transform_frame_to_root.actionInverse @ configuration.get_frame_jacobian(self.root)


def perturbed(pose: pin.SE3, rng: np.random.Generator, shift: float, turn: float) -> pin.SE3:
    """`pose` moved by `shift` [m] and turned by `turn` [rad], directions random."""
    direction = rng.normal(size=3)
    axis = rng.normal(size=3)
    out = pose.copy()
    out.translation = pose.translation + shift * direction / np.linalg.norm(direction)
    out.rotation = pose.rotation @ pin.exp3(turn * axis / np.linalg.norm(axis))
    return out


def solve_case(problem: Problem, model: pin.Model, tier: int, rng: np.random.Generator) -> dict:
    """One random problem instance and pink's solution of it."""
    lower, upper = model.lowerPositionLimit, model.upperPositionLimit
    span = upper - lower
    q = lower + span * rng.uniform(0.2, 0.8, size=model.nq)
    # Two joints a hair inside a position limit, with the posture target on
    # the far side of it: the one-step position box is active there.
    q_posture = lower + span * rng.uniform(0.3, 0.7, size=model.nq)
    for joint in rng.choice(model.nq, size=2, replace=False):
        if rng.uniform() < 0.5:
            q[joint] = upper[joint] - 1e-4 * span[joint]
            q_posture[joint] = upper[joint]
        else:
            q[joint] = lower[joint] + 1e-4 * span[joint]
            q_posture[joint] = lower[joint]

    data = model.createData()
    configuration = pink.Configuration(model, data, q)
    shift, turn = (1e-3, 1e-3) if tier == 1 else (0.2, 0.5)
    target_world = perturbed(
        configuration.get_transform_frame_to_world(problem.world_frame), rng, shift, turn
    )
    target_rel = perturbed(
        configuration.get_transform(problem.rel_frame, problem.rel_root), rng, shift, turn
    )

    frame_cls = FrameTask if tier == 1 else FirstOrderFrameTask
    rel_cls = RelativeFrameTask if tier == 1 else FirstOrderRelativeFrameTask
    world_task = frame_cls(
        problem.world_frame, position_cost=WORLD_COST, orientation_cost=WORLD_COST, gain=WORLD_GAIN
    )
    world_task.set_target(target_world)
    rel_task = rel_cls(
        problem.rel_frame,
        problem.rel_root,
        position_cost=REL_COST,
        orientation_cost=REL_COST,
        gain=REL_GAIN,
    )
    rel_task.set_target(target_rel)
    posture_task = PostureTask(cost=POSTURE_COST, gain=POSTURE_GAIN)
    posture_task.set_target(q_posture)

    limits = [ConfigurationLimit(model, config_limit_gain=1.0), VelocityLimit(model)]
    v = pink.solve_ik(
        configuration,
        [world_task, rel_task, posture_task],
        DT,
        solver="proxqp",
        damping=DAMPING,
        limits=limits,
        eps_abs=1e-12,
        eps_rel=0.0,
        max_iter=10000,
    )
    dq = v * DT
    dq_hi = np.minimum(upper - q, DT * model.velocityLimit)
    dq_lo = np.maximum(lower - q, -DT * model.velocityLimit)
    assert np.all(dq <= dq_hi + 1e-9) and np.all(dq >= dq_lo - 1e-9), "pink left its own box"
    active = int(np.sum(np.abs(dq - dq_hi) < ACTIVE_TOL) + np.sum(np.abs(dq - dq_lo) < ACTIVE_TOL))
    return {
        "q": q,
        "q_posture": q_posture,
        "target_world": target_world,
        "target_rel": target_rel,
        "v": v,
        "active": active,
    }


def fmt(values: np.ndarray, width: int) -> str:
    """`values` padded with zeros to `width`, as a C++ brace list."""
    padded = np.zeros(width)
    padded[: len(values)] = values
    return "{" + ", ".join(float(x).hex() for x in padded) + "}"


def fmt_pose(pose: pin.SE3) -> str:
    """Rotation (row-major) then translation, as a C++ brace list."""
    return fmt(np.concatenate([pose.rotation.reshape(9), pose.translation]), 12)


def main() -> None:
    """Solve every case and write the fragment."""
    lines = [
        "// Generated by rtc_tsid/test/golden/gen_pink_reference.py. Do not edit.",
        "// pink's joint velocities for the two-frame problems of",
        "// test_clik_pink_reference.cpp - see the generator's docstring for the",
        "// mapping, the two tiers and how to re-record.",
        "//",
    ]
    for package in ("pin-pink", "pin", "qpsolvers", "proxsuite", "numpy"):
        lines.append(f"//   {package} {importlib.metadata.version(package)}")
    lines += [
        "// clang-format off",
        f"constexpr double kPinkDt = {DT!r};",
        f"constexpr double kPinkDamping = {DAMPING!r};",
        f"constexpr double kPinkWorldCost = {WORLD_COST!r};",
        f"constexpr double kPinkWorldGain = {WORLD_GAIN!r};",
        f"constexpr double kPinkRelCost = {REL_COST!r};",
        f"constexpr double kPinkRelGain = {REL_GAIN!r};",
        f"constexpr double kPinkPostureCost = {POSTURE_COST!r};",
        f"constexpr double kPinkPostureGain = {POSTURE_GAIN!r};",
        "// {problem, tier, nv, active limit rows in pink's solution,",
        "//  q, q_posture, world target, relative target (rotation row-major,",
        "//  translation), pink's v} - vectors padded with zeros to 17.",
        "constexpr PinkCase kPinkCases[] = {",
    ]
    total_active = {1: 0, 2: 0}
    for problem_idx, problem in enumerate(PROBLEMS):
        model = pin.buildModelFromUrdf(str(problem.urdf))
        assert model.nq == model.nv <= MAX_NV, problem.name
        for tier in (1, 2):
            rng = np.random.default_rng(20261008 + 100 * problem_idx + tier)
            for _ in range(CASES_PER_TIER):
                case = solve_case(problem, model, tier, rng)
                total_active[tier] += case["active"]
                lines.append(
                    f"  {{{problem_idx}, {tier}, {model.nv}, {case['active']},\n"
                    f"   {fmt(case['q'], MAX_NV)},\n"
                    f"   {fmt(case['q_posture'], MAX_NV)},\n"
                    f"   {fmt_pose(case['target_world'])},\n"
                    f"   {fmt_pose(case['target_rel'])},\n"
                    f"   {fmt(case['v'], MAX_NV)}}},"
                )
    lines += ["};", "// clang-format on", ""]
    assert all(n > 0 for n in total_active.values()), "no case has an active limit"
    OUT.write_text("\n".join(lines))
    print(f"wrote {OUT} - active limit rows per tier: {total_active}")


if __name__ == "__main__":
    main()
