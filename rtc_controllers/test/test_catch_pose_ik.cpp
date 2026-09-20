// ── dynamic_catching S1.9: catch-pose IK + catchability gate ────────────────
// Suite for rtc::catching::CatchPoseIk (L3 §4.2, gates G3-G / G3-I / G3-K).
//
// Two fixtures, and the pair is the point:
//   • serial_6r_wrist — nv = 6, so the 5-row task leaves EXACTLY one free
//     degree of freedom. A dense sweep over roll therefore enumerates the whole
//     solution family, which is what makes the null-space ascent checkable
//     against something other than itself (RollSweep below).
//   • serial_7dof — nv = 7, redundant beyond the roll, so the same code runs
//     with a 2-dimensional null space and a non-square Jacobian.
// serial_6dof (all-Z axes) appears once, as the rank-deficient NEGATIVE
// fixture: its LOCAL angular rows about x and y are structurally zero, so J₅
// can never have rank 5 and w₅ is exactly the degenerate case the fail-closed
// rule exists for.
//
// ── How the oracles avoid measuring the implementation against itself ───────
// Three independent levels, because the cheap one is the one that can agree
// with a wrong implementation:
//
//   1. StackJacobianRef() re-derives J from the same two GetFrameJacobian
//      calls. That pins arithmetic, NOT the row selection — it would agree with
//      a version that picked the wrong rows. It is here for the det/LDLT
//      cross-check only.
//   2. FdJacobian() builds J from finite differences of FORWARD KINEMATICS
//      alone: positions differenced in world, orientations differenced as
//      log3(R(q−h)ᵀR(q+h))/2h. It touches no Jacobian API, so it is what
//      actually fixes the frame and row convention.
//   3. RollSweep() solves a full 6-DoF IK at each of 360 roll angles and reads
//      w₅ off the resulting branch. It never calls the gradient code, so it is
//      an oracle for the ASCENT independent of how the gradient is computed.
//
// ── Reading the last iterate ────────────────────────────────────────────────
// Several tests need "the iterate after exactly n steps", which the result POD
// does not expose directly — q* is the last ACCEPTED iterate. LastIterate()
// gets it anyway, without a debug hook, by making acceptance vacuous
// (eps_pos huge, alpha_max = π) and the ascent stopping condition unreachable
// (k_manip > 0 with manip_grad_tol = 0). Every iterate is then accepted, the
// loop always runs to max_iter, and q* is the max_iter-th iterate. Differencing
// it at n and n+1 measures ONE step of the update law, which is how the
// per-iteration step bound is checked directly rather than inferred.
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_controllers/catching/catch_pose_ik.hpp"
#include "rtc_controllers/compliance/differential_ik.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/catch_arm_fixture.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <numbers>
#include <random>
#include <span>
#include <string>
#include <vector>

namespace {

using rtc::catching::CatchPoseIk;
using rtc::catching::CatchPoseIkOptions;
using rtc::catching::CatchPoseIkResult;
using rtc::catching::CatchPoseReason;
using rtc::catching::ManipDefinition;

using rtc::testing::Arm;
using rtc::testing::Arm6R;
using rtc::testing::Arm7R;
using rtc::testing::AsSpan;
using rtc::testing::FdJacobian;
using rtc::testing::Fk;
using rtc::testing::MakeArm;
using rtc::testing::ManipFromRows;
using rtc::testing::Pose;
using rtc::testing::SampleQ;
using rtc::testing::StackJacobianRef;
using rtc::testing::Target;
using rtc::testing::TargetAt;

// ── Options ─────────────────────────────────────────────────────────────────

[[nodiscard]] CatchPoseIkOptions BaseOptions() {
  CatchPoseIkOptions o;
  o.max_iter = 200;
  o.eps_pos = 1e-4;
  o.alpha_max = 1e-3;
  o.dq_step_max = 0.2;
  o.manipulability_min = 0.0;  // gate off unless a test asks for it
  return o;
}

/// See the file header: makes acceptance vacuous and the ascent stop
/// unreachable, so q* is the max_iter-th iterate.
[[nodiscard]] CatchPoseIkOptions LastIterateOptions(int iterations) {
  CatchPoseIkOptions o = BaseOptions();
  o.max_iter = iterations;
  o.eps_pos = 1e9;
  o.alpha_max = std::numbers::pi;
  o.k_manip = 1e-9;        // nonzero so the stopping test is evaluated…
  o.manip_grad_tol = 0.0;  // …and ‖·‖ < 0 is never true
  return o;
}

[[nodiscard]] Eigen::VectorXd ResultQ(const CatchPoseIkResult& r) {
  Eigen::VectorXd q(r.nv);
  for (int i = 0; i < r.nv; ++i)
    q(i) = r.q[static_cast<std::size_t>(i)];
  return q;
}

// ── Level-3 oracle: full 6-DoF IK, used to sweep roll ───────────────────────

/// Solve position + full orientation onto (p_d, R_d) from `q`, in place.
/// Independent of CatchPoseIk: 6 rows, no null-space term, no manipulability.
[[nodiscard]] bool Ik6(Arm& a, const Eigen::Vector3d& p_d, const Eigen::Matrix3d& r_d,
                       Eigen::VectorXd& q, int iters = 400, double tol = 1e-9) {
  rtc::compliance::DifferentialIk dls;
  dls.Resize(a.nv, 6);
  Eigen::MatrixXd j_world = Eigen::MatrixXd::Zero(6, a.nv);
  Eigen::VectorXd err(6);
  Eigen::VectorXd dq(a.nv);
  for (int k = 0; k < iters; ++k) {
    a.handle->ComputeJacobians(AsSpan(q));
    a.handle->GetFrameJacobian(a.frame, pinocchio::LOCAL_WORLD_ALIGNED, j_world);
    const Eigen::Vector3d p = a.handle->GetFramePosition(a.frame);
    const Eigen::Matrix3d r = a.handle->GetFrameRotation(a.frame);
    err.head(3) = p_d - p;
    // World-aligned rotation error, matching the LWA angular rows above.
    err.tail(3) = rtc::math::se3::log3(r_d * r.transpose());
    if (err.head(3).norm() < tol && err.tail(3).norm() < tol)
      return true;
    const rtc::compliance::DifferentialIk::Result res = dls.Compute(j_world, 1e-4, 1e-3);
    if (!res.ok)
      return false;
    dls.Solve(err, dq);
    const double step = dq.lpNorm<Eigen::Infinity>();
    if (step > 0.1)
      dq *= (0.1 / step);
    q += dq;
  }
  return err.head(3).norm() < 1e-7 && err.tail(3).norm() < 1e-7;
}

struct RollSample {
  double psi{0.0};
  double w5{0.0};
  bool ok{false};
};

/// w₅ along the one-parameter family of poses the 5-row task leaves free.
///
/// R_d(ψ) = R(q0)·Rz(ψ) keeps the frame's local +z (hence the approach axis)
/// fixed while spinning the hand about it, so every ψ is a legal catch pose for
/// the SAME target. Solutions are continued from the previous ψ so the samples
/// stay on one IK branch — the branch the seed q0 sits on, which is the only
/// one the local ascent could reach.
[[nodiscard]] std::vector<RollSample> RollSweep(Arm& a, const Eigen::VectorXd& q0, int half_steps,
                                                double d_psi) {
  const Pose base = Fk(a, q0);
  std::vector<RollSample> out;
  out.resize(static_cast<std::size_t>(2 * half_steps + 1));

  const auto solve_side = [&](int sign) {
    Eigen::VectorXd q = q0;
    for (int s = 1; s <= half_steps; ++s) {
      const double psi = sign * s * d_psi;
      const Eigen::Matrix3d r_d =
          base.R * Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      RollSample sample;
      sample.psi = psi;
      sample.ok = Ik6(a, base.p, r_d, q);
      if (sample.ok)
        sample.w5 = ManipFromRows(StackJacobianRef(a, q), 5);
      out[static_cast<std::size_t>(half_steps + sign * s)] = sample;
      if (!sample.ok)
        break;  // branch lost; the rest of this side is unusable
    }
  };

  RollSample centre;
  centre.psi = 0.0;
  centre.ok = true;
  centre.w5 = ManipFromRows(StackJacobianRef(a, q0), 5);
  out[static_cast<std::size_t>(half_steps)] = centre;
  solve_side(+1);
  solve_side(-1);
  return out;
}

/// The largest w₅ reachable from ψ = 0 by walking uphill along the sampled
/// branch — the local maximum a gradient ascent from q0 can actually attain.
[[nodiscard]] double LocalMaxFromCentre(const std::vector<RollSample>& s) {
  const auto n = static_cast<int>(s.size());
  const int centre = n / 2;
  double best = s[static_cast<std::size_t>(centre)].w5;
  for (const int dir : {+1, -1}) {
    for (int i = centre + dir; i >= 0 && i < n; i += dir) {
      const RollSample& cur = s[static_cast<std::size_t>(i)];
      if (!cur.ok || cur.w5 < best)
        break;  // stop at the first descent — this is a LOCAL max
      best = cur.w5;
    }
  }
  return best;
}

}  // namespace

// ── 1. Convergence and step limiting (SPRINT 1) ──────────────────────────────

TEST(CatchPoseIk, ConvergesOnBothFixtures) {
  for (const bool six : {true, false}) {
    Arm a = six ? Arm6R() : Arm7R();
    CatchPoseIk ik;
    ik.Resize(a.nv);
    std::mt19937 rng(20260920U);

    int solved = 0;
    constexpr int kCases = 30;
    for (int c = 0; c < kCases; ++c) {
      const Eigen::VectorXd q_goal = SampleQ(a, rng);
      const Target t = TargetAt(a, q_goal);
      // Seed displaced from the goal, so the solver has real work to do while
      // the target stays reachable by construction.
      Eigen::VectorXd seed = q_goal;
      for (int i = 0; i < a.nv; ++i)
        seed(i) += (i % 2 == 0 ? 0.20 : -0.15);

      const CatchPoseIkResult r =
          ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, BaseOptions());
      if (r.reason != CatchPoseReason::kNone)
        continue;
      ++solved;
      EXPECT_LT(r.pos_error, BaseOptions().eps_pos) << "case " << c;
      EXPECT_LE(r.theta, BaseOptions().alpha_max) << "case " << c;
      EXPECT_TRUE(r.accepted);
    }
    // Not a rate gate (that is G3-G, measured on real candidates) — just the
    // assertion that reachable targets are overwhelmingly solved, so a later
    // regression that breaks the loop cannot hide behind "IK is hard".
    EXPECT_GE(solved, kCases - 3) << (six ? "6R" : "7R");
  }
}

TEST(CatchPoseIk, AcceptedPoseRespectsJointLimits) {
  Arm a = Arm6R();
  CatchPoseIk ik;
  ik.Resize(a.nv);
  std::mt19937 rng(7U);

  for (int c = 0; c < 40; ++c) {
    const Eigen::VectorXd q_goal = SampleQ(a, rng, 0.05);
    const Target t = TargetAt(a, q_goal);
    // Seed pressed hard against the lower bounds, so the first steps travel
    // along the limits rather than through open space.
    Eigen::VectorXd seed(a.nv);
    for (int i = 0; i < a.nv; ++i)
      seed(i) = a.model->lowerPositionLimit(i) + 0.01;

    const CatchPoseIkResult r = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, BaseOptions());
    if (r.reason != CatchPoseReason::kNone && r.reason != CatchPoseReason::kBelowManipMin)
      continue;
    const Eigen::VectorXd q = ResultQ(r);
    for (int i = 0; i < a.nv; ++i) {
      EXPECT_GE(q(i), a.model->lowerPositionLimit(i)) << "case " << c << " joint " << i;
      EXPECT_LE(q(i), a.model->upperPositionLimit(i)) << "case " << c << " joint " << i;
    }
  }
}

TEST(CatchPoseIk, EveryStepObeysTheInfinityNormBound) {
  Arm a = Arm6R();
  CatchPoseIk ik;
  ik.Resize(a.nv);
  std::mt19937 rng(11U);

  const Eigen::VectorXd q_goal = SampleQ(a, rng);
  const Target t = TargetAt(a, q_goal);
  // A seed far from the goal: without a bound the first DLS step would be huge.
  Eigen::VectorXd seed = Eigen::VectorXd::Zero(a.nv);

  constexpr double kStepMax = 0.05;
  Eigen::VectorXd prev = seed;
  for (int n = 1; n <= 12; ++n) {
    CatchPoseIkOptions o = LastIterateOptions(n);
    o.dq_step_max = kStepMax;
    const CatchPoseIkResult r = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, o);
    ASSERT_EQ(r.reason, CatchPoseReason::kNone) << "n = " << n;
    const Eigen::VectorXd cur = ResultQ(r);
    // One update-law step, isolated. A clamp removed from the production step
    // makes the very first difference exceed the bound.
    EXPECT_LE((cur - prev).lpNorm<Eigen::Infinity>(), kStepMax * (1.0 + 1e-9)) << "step " << n;
    prev = cur;
  }
}

TEST(CatchPoseIk, TighterStepBoundCostsMoreIterations) {
  Arm a = Arm6R();
  CatchPoseIk ik;
  ik.Resize(a.nv);
  std::mt19937 rng(3U);

  const Eigen::VectorXd q_goal = SampleQ(a, rng);
  const Target t = TargetAt(a, q_goal);
  Eigen::VectorXd seed = Eigen::VectorXd::Zero(a.nv);

  // Both budgets are generous enough to converge; what differs is how many
  // iterations each needs, which is the quantity under test. (With the shared
  // 200-iteration default the tight run simply cannot travel far enough — it
  // reports kNotConverged, which would make this a test of the budget rather
  // than of the bound.)
  CatchPoseIkOptions loose = BaseOptions();
  loose.max_iter = 3000;
  loose.dq_step_max = 0.5;
  CatchPoseIkOptions tight = BaseOptions();
  tight.max_iter = 3000;
  tight.dq_step_max = 0.01;

  const CatchPoseIkResult r_loose = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, loose);
  const CatchPoseIkResult r_tight = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, tight);
  ASSERT_EQ(r_loose.reason, CatchPoseReason::kNone);
  ASSERT_EQ(r_tight.reason, CatchPoseReason::kNone);
  // The bound is not cosmetic: it is what the iteration count is spent on.
  EXPECT_GT(r_tight.iterations, r_loose.iterations);
}

// ── 2. Null-space ascent on w₅ (SPRINT 2) ────────────────────────────────────

TEST(CatchPoseIk, AscentReachesTheRollSweepLocalMaximum) {
  Arm a = Arm6R();
  CatchPoseIk ik;
  ik.Resize(a.nv);
  std::mt19937 rng(101U);

  int checked = 0;
  int unconverged = 0;
  for (int c = 0; c < 16 && checked < 4; ++c) {
    const Eigen::VectorXd seed = SampleQ(a, rng, 0.5);
    const Target t = TargetAt(a, seed);  // seed already satisfies the task

    CatchPoseIkOptions o = BaseOptions();
    o.max_iter = 1500;
    o.k_manip = 0.05;
    o.manip_grad_tol = 1e-5;
    o.dq_step_max = 0.02;
    const CatchPoseIkResult r = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, o);
    if (r.reason != CatchPoseReason::kNone || !r.manip_converged) {
      ++unconverged;
      continue;
    }

    // 1° grid over ±90°: fine enough that the sampled maximum is within the
    // tolerance below, wide enough to contain the nearest maximum.
    const std::vector<RollSample> sweep = RollSweep(a, seed, 90, std::numbers::pi / 180.0);
    const double local_max = LocalMaxFromCentre(sweep);
    ASSERT_GT(local_max, 0.0);

    // The ascent must reach the branch's local maximum, and must not exceed it
    // — overshooting would mean it left the constraint surface.
    EXPECT_GT(r.w5, 0.97 * local_max) << "case " << c;
    EXPECT_LT(r.w5, 1.02 * local_max) << "case " << c;
    ++checked;
  }
  EXPECT_GE(checked, 3) << "too few usable sweeps to call this measured (" << unconverged
                        << " candidates did not converge)";
}

TEST(CatchPoseIk, AscentNeverLowersManipulabilityVersusTheBaseline) {
  for (const bool six : {true, false}) {
    Arm a = six ? Arm6R() : Arm7R();
    CatchPoseIk ik;
    ik.Resize(a.nv);
    std::mt19937 rng(202U);

    int compared = 0;
    for (int c = 0; c < 20; ++c) {
      const Eigen::VectorXd seed = SampleQ(a, rng, 0.4);
      const Target t = TargetAt(a, seed);

      CatchPoseIkOptions off = BaseOptions();
      off.max_iter = 400;
      off.dq_step_max = 0.02;
      CatchPoseIkOptions on = off;
      on.k_manip = 0.05;
      on.manip_grad_tol = 1e-5;

      const CatchPoseIkResult r_off = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, off);
      const CatchPoseIkResult r_on = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, on);
      if (r_off.reason != CatchPoseReason::kNone || r_on.reason != CatchPoseReason::kNone)
        continue;
      ++compared;
      // k_manip = 0 is the original [확정 D-18] behaviour. Turning the term on
      // may not make the pose worse conditioned.
      EXPECT_GE(r_on.w5, r_off.w5 * (1.0 - 1e-9)) << (six ? "6R" : "7R") << " case " << c;
    }
    EXPECT_GE(compared, 15) << (six ? "6R" : "7R");
  }
}

TEST(CatchPoseIk, ReversingTheGradientSignDescendsInstead) {
  Arm a = Arm6R();
  CatchPoseIk ik;
  ik.Resize(a.nv);
  std::mt19937 rng(303U);

  int compared = 0;
  for (int c = 0; c < 12; ++c) {
    const Eigen::VectorXd seed = SampleQ(a, rng, 0.4);
    const Target t = TargetAt(a, seed);

    CatchPoseIkOptions up = BaseOptions();
    up.max_iter = 300;
    up.dq_step_max = 0.02;
    up.k_manip = 0.05;
    up.manip_grad_tol = 1e-5;
    CatchPoseIkOptions down = up;
    down.k_manip = -0.05;  // the sign-flip mutant, expressed as an input

    const CatchPoseIkResult r_up = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, up);
    const CatchPoseIkResult r_down = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, down);
    if (r_up.reason != CatchPoseReason::kNone || r_down.reason != CatchPoseReason::kNone)
      continue;
    ++compared;
    // Directional, not merely "different": a term that moved the pose without
    // following the gradient would fail this as often as it passed.
    EXPECT_GT(r_up.w5, r_down.w5) << "case " << c;
  }
  EXPECT_GE(compared, 8);
}

TEST(CatchPoseIk, TaskToleranceHoldsWhileTheAscentRuns) {
  Arm a = Arm6R();
  CatchPoseIk ik;
  ik.Resize(a.nv);
  std::mt19937 rng(404U);

  for (int c = 0; c < 12; ++c) {
    const Eigen::VectorXd seed = SampleQ(a, rng, 0.4);
    const Target t = TargetAt(a, seed);
    CatchPoseIkOptions o = BaseOptions();
    o.max_iter = 300;
    o.dq_step_max = 0.02;
    o.k_manip = 0.05;
    o.manip_grad_tol = 1e-5;

    const CatchPoseIkResult r = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, o);
    if (r.reason != CatchPoseReason::kNone)
      continue;
    // The strict-priority claim: the returned pose still meets the primary
    // task. Drop the null-space projector and the ascent starts dragging the
    // position/axis residual out of tolerance.
    EXPECT_LT(r.pos_error, o.eps_pos) << "case " << c;
    EXPECT_LE(r.theta, o.alpha_max) << "case " << c;
  }
}

TEST(CatchPoseIk, FiniteDifferenceGradientIsStepSizeIndependent) {
  Arm a = Arm6R();
  CatchPoseIk ik;
  ik.Resize(a.nv);
  std::mt19937 rng(505U);

  for (int c = 0; c < 6; ++c) {
    const Eigen::VectorXd seed = SampleQ(a, rng, 0.4);
    const Target t = TargetAt(a, seed);
    CatchPoseIkOptions o = BaseOptions();
    o.max_iter = 300;
    o.dq_step_max = 0.02;
    o.k_manip = 0.05;
    o.manip_grad_tol = 1e-5;

    CatchPoseIkOptions half = o;
    half.fd_step = o.fd_step / 2.0;

    const CatchPoseIkResult r_h = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, o);
    const CatchPoseIkResult r_h2 = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, half);
    if (r_h.reason != CatchPoseReason::kNone || r_h2.reason != CatchPoseReason::kNone)
      continue;
    // Halving h must not move the answer: if it does, h is in the regime where
    // truncation or cancellation dominates and the gradient is noise.
    EXPECT_NEAR(r_h.w5, r_h2.w5, 1e-4 * std::max(1.0, r_h.w5)) << "case " << c;
  }
}

TEST(CatchPoseIk, RepeatedAndInterleavedSolvesAreBitIdentical) {
  Arm a = Arm6R();
  CatchPoseIk ik;
  ik.Resize(a.nv);
  std::mt19937 rng(606U);

  const Eigen::VectorXd seed = SampleQ(a, rng, 0.4);
  const Target t = TargetAt(a, seed);
  const Eigen::VectorXd other_seed = SampleQ(a, rng, 0.4);
  const Target other = TargetAt(a, other_seed);

  CatchPoseIkOptions o = BaseOptions();
  o.max_iter = 200;
  o.dq_step_max = 0.02;
  o.k_manip = 0.05;
  o.manip_grad_tol = 1e-5;

  const CatchPoseIkResult first = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, o);
  const CatchPoseIkResult again = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, o);
  // A DIFFERENT candidate in between. This is the leak a warm-started solver
  // would show: same inputs, different answer, because the previous problem's
  // state survived. The map and the planner would then disagree purely on
  // search order (plan §11).
  const CatchPoseIkResult unrelated =
      ik.Solve(*a.handle, a.frame, other.p_c, other.v_ball, other_seed, o);
  const CatchPoseIkResult third = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, o);

  ASSERT_EQ(first.reason, CatchPoseReason::kNone);
  ASSERT_EQ(unrelated.nv, a.nv);
  for (int i = 0; i < first.nv; ++i) {
    const auto u = static_cast<std::size_t>(i);
    EXPECT_EQ(first.q[u], again.q[u]) << "joint " << i;
    EXPECT_EQ(first.q[u], third.q[u]) << "joint " << i << " after an interleaved candidate";
  }
  EXPECT_EQ(first.w5, third.w5);
  EXPECT_EQ(first.iterations, third.iterations);
}

// ── 3. w₅ / w₆ against independent oracles (SPRINT 4) ────────────────────────

TEST(CatchPoseIk, ManipulabilityMatchesADenseDeterminant) {
  for (const bool six : {true, false}) {
    Arm a = six ? Arm6R() : Arm7R();
    CatchPoseIk ik;
    ik.Resize(a.nv);
    std::mt19937 rng(707U);

    for (int c = 0; c < 15; ++c) {
      const Eigen::VectorXd seed = SampleQ(a, rng, 0.4);
      const Target t = TargetAt(a, seed);
      const CatchPoseIkResult r =
          ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, BaseOptions());
      if (r.reason != CatchPoseReason::kNone)
        continue;

      const Eigen::MatrixXd j = StackJacobianRef(a, ResultQ(r));
      EXPECT_NEAR(r.w5, ManipFromRows(j, 5), 1e-9 * std::max(1.0, r.w5)) << "case " << c;
      EXPECT_NEAR(r.w6, ManipFromRows(j, 6), 1e-9 * std::max(1.0, r.w6)) << "case " << c;
      EXPECT_TRUE(r.w5_valid);
      EXPECT_TRUE(r.w6_valid);
    }
  }
}

TEST(CatchPoseIk, ManipulabilityMatchesAFiniteDifferenceJacobian) {
  Arm a = Arm6R();
  CatchPoseIk ik;
  ik.Resize(a.nv);
  std::mt19937 rng(808U);

  for (int c = 0; c < 8; ++c) {
    const Eigen::VectorXd seed = SampleQ(a, rng, 0.4);
    const Target t = TargetAt(a, seed);
    const CatchPoseIkResult r = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, BaseOptions());
    if (r.reason != CatchPoseReason::kNone)
      continue;

    const Eigen::VectorXd q = ResultQ(r);
    const double w5_fd = ManipFromRows(FdJacobian(a, q), 5);
    // Loose next to the dense-determinant check, because this oracle carries
    // the finite-difference error — and that is the point: it uses NO Jacobian
    // routine, so it is what pins the frame and row convention rather than
    // repeating it.
    EXPECT_NEAR(r.w5, w5_fd, 1e-5 * std::max(1.0, r.w5)) << "case " << c;
  }
}

TEST(CatchPoseIk, AWrongRowOrFrameChoiceWouldChangeTheAnswer) {
  Arm a = Arm6R();
  CatchPoseIk ik;
  ik.Resize(a.nv);
  std::mt19937 rng(909U);

  const Eigen::VectorXd seed = SampleQ(a, rng, 0.4);
  const Target t = TargetAt(a, seed);
  const CatchPoseIkResult r = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, BaseOptions());
  ASSERT_EQ(r.reason, CatchPoseReason::kNone);
  const Eigen::VectorXd q = ResultQ(r);

  Eigen::MatrixXd j_world = Eigen::MatrixXd::Zero(6, a.nv);
  Eigen::MatrixXd j_local = Eigen::MatrixXd::Zero(6, a.nv);
  a.handle->ComputeJacobians(AsSpan(q));
  a.handle->GetFrameJacobian(a.frame, pinocchio::LOCAL_WORLD_ALIGNED, j_world);
  a.handle->GetFrameJacobian(a.frame, pinocchio::LOCAL, j_local);

  // Mutant 1: angular rows taken world-aligned instead of LOCAL.
  Eigen::MatrixXd all_world(6, a.nv);
  all_world.topRows(3) = j_world.topRows(3);
  all_world.bottomRows(3) = j_world.bottomRows(3);
  EXPECT_GT(std::abs(ManipFromRows(all_world, 5) - r.w5), 1e-6 * std::max(1.0, r.w5));

  // Mutant 2: the roll row kept and an alignment row dropped (rows 4,5 instead
  // of 3,4 for the angular block).
  Eigen::MatrixXd roll_row(5, a.nv);
  roll_row.topRows(3) = j_world.topRows(3);
  roll_row.row(3) = j_local.row(4);
  roll_row.row(4) = j_local.row(5);
  EXPECT_GT(std::abs(ManipFromRows(roll_row, 5) - r.w5), 1e-6 * std::max(1.0, r.w5));

  // NOT a mutant: taking the LINEAR rows LOCAL instead of world-aligned leaves
  // w₅ unchanged, and no tolerance will ever separate the two. The two stacks
  // differ by T = diag(R, I₂), so their Gram matrices differ by T(·)Tᵀ and the
  // determinants by det(R)² = 1 — the measurement is invariant to that choice
  // by construction, measured here rather than assumed.
  //
  // That invariance is exactly why w₅ CANNOT pin the linear frame, and it is
  // worth being explicit about: the residual can, because p_c − p_C is a world
  // vector and multiplying it by a LOCAL Jacobian gives a step in the wrong
  // direction. ConvergesOnBothFixtures is what holds that convention — this
  // assertion documents why it has to be that test and not this one.
  Eigen::MatrixXd local_linear(6, a.nv);
  local_linear.topRows(3) = j_local.topRows(3);
  local_linear.bottomRows(3) = j_local.bottomRows(3);
  EXPECT_NEAR(ManipFromRows(local_linear, 5), r.w5, 1e-9 * std::max(1.0, r.w5));
}

TEST(CatchPoseIk, TheSolutionDoesNotDependOnTheGateDefinition) {
  Arm a = Arm6R();
  CatchPoseIk ik;
  ik.Resize(a.nv);
  std::mt19937 rng(111U);

  const Eigen::VectorXd seed = SampleQ(a, rng, 0.4);
  const Target t = TargetAt(a, seed);
  CatchPoseIkOptions five = BaseOptions();
  five.max_iter = 200;
  five.dq_step_max = 0.02;
  five.k_manip = 0.05;
  five.manip_grad_tol = 1e-5;
  CatchPoseIkOptions six = five;
  six.definition = ManipDefinition::kArm6Row;

  const CatchPoseIkResult r5 = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, five);
  const CatchPoseIkResult r6 = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, six);
  ASSERT_EQ(r5.reason, CatchPoseReason::kNone);
  ASSERT_EQ(r6.reason, CatchPoseReason::kNone);
  // Q3a: the ascent climbs w₅ whichever definition is selected, so both runs
  // return the SAME pose and w₅/w₆ at it are two readings of one configuration.
  // If this ever fails, the C-3 comparison of the two distributions is no
  // longer comparing like with like.
  for (int i = 0; i < r5.nv; ++i)
    EXPECT_EQ(r5.q[static_cast<std::size_t>(i)], r6.q[static_cast<std::size_t>(i)]);
  EXPECT_EQ(r5.w5, r6.w5);
  EXPECT_EQ(r5.w6, r6.w6);
}

// ── 4. Reason codes (SPRINT 5) ───────────────────────────────────────────────

TEST(CatchPoseIk, RejectionsCarryTheirOwnReason) {
  Arm a = Arm6R();
  CatchPoseIk ik;
  ik.Resize(a.nv);
  std::mt19937 rng(222U);

  const Eigen::VectorXd seed = SampleQ(a, rng, 0.4);
  const Target t = TargetAt(a, seed);
  const CatchPoseIkOptions ok = BaseOptions();
  const double nan = std::numeric_limits<double>::quiet_NaN();

  // Sanity: the unperturbed problem is accepted, so every rejection below is
  // caused by the one thing that was changed (a negative test that passes for
  // an unrelated reason measures nothing).
  ASSERT_EQ(ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, ok).reason, CatchPoseReason::kNone);

  {
    CatchPoseIkOptions bad = ok;
    bad.eps_pos = 0.0;
    EXPECT_EQ(ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, bad).reason,
              CatchPoseReason::kOptionsInvalid);
  }
  {
    CatchPoseIkOptions bad = ok;
    bad.rho = nan;
    EXPECT_EQ(ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, bad).reason,
              CatchPoseReason::kOptionsInvalid);
  }
  {
    Eigen::VectorXd bad_seed = seed;
    bad_seed(2) = nan;
    EXPECT_EQ(ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, bad_seed, ok).reason,
              CatchPoseReason::kSeedNonFinite);
  }
  {
    const Eigen::VectorXd short_seed = Eigen::VectorXd::Zero(a.nv - 1);
    EXPECT_EQ(ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, short_seed, ok).reason,
              CatchPoseReason::kSeedNonFinite);
  }
  {
    Eigen::Vector3d bad_p = t.p_c;
    bad_p.y() = nan;
    EXPECT_EQ(ik.Solve(*a.handle, a.frame, bad_p, t.v_ball, seed, ok).reason,
              CatchPoseReason::kTargetNonFinite);
  }
  {
    Eigen::Vector3d bad_v = t.v_ball;
    bad_v.z() = std::numeric_limits<double>::infinity();
    EXPECT_EQ(ik.Solve(*a.handle, a.frame, t.p_c, bad_v, seed, ok).reason,
              CatchPoseReason::kVelocityNonFinite);
  }
  {
    // A stationary ball has no approach axis. This is a rejection, not a
    // clamped unit vector (NUM-7) — the whole point of the v_eps key.
    EXPECT_EQ(ik.Solve(*a.handle, a.frame, t.p_c, Eigen::Vector3d::Zero(), seed, ok).reason,
              CatchPoseReason::kSpeedTooLow);
    const Eigen::Vector3d creeping = 1e-9 * t.v_ball.normalized();
    EXPECT_EQ(ik.Solve(*a.handle, a.frame, t.p_c, creeping, seed, ok).reason,
              CatchPoseReason::kSpeedTooLow);
  }
  {
    // Far outside the workspace.
    const Eigen::Vector3d unreachable(50.0, 50.0, 50.0);
    EXPECT_EQ(ik.Solve(*a.handle, a.frame, unreachable, t.v_ball, seed, ok).reason,
              CatchPoseReason::kNotConverged);
  }
  {
    CatchPoseIkOptions gate = ok;
    gate.manipulability_min = 1e9;
    const CatchPoseIkResult r = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, gate);
    EXPECT_EQ(r.reason, CatchPoseReason::kBelowManipMin);
    EXPECT_FALSE(r.accepted);
    // The pose is still reported: the IK succeeded and only the gate refused
    // it, which is what the catchability map records.
    EXPECT_GT(r.w5, 0.0);
    EXPECT_EQ(r.nv, a.nv);
  }
  {
    CatchPoseIk unsized;
    unsized.Resize(0);
    EXPECT_EQ(unsized.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, ok).reason,
              CatchPoseReason::kModelInvalid);
  }
  {
    EXPECT_EQ(ik.Solve(*a.handle, 0, t.p_c, t.v_ball, seed, ok).reason,
              CatchPoseReason::kModelInvalid);
  }
}

TEST(CatchPoseIk, AStructurallySingularArmIsRejectedNotScoredZero) {
  // serial_6dof has all six axes parallel to Z, so the LOCAL angular rows about
  // x and y vanish identically: J₅ has rank ≤ 3 and J₅J₅ᵀ is singular by
  // construction. Acceptance is made vacuous so the run reaches the gate — the
  // point is what the GATE does with a degenerate Gram matrix, not whether this
  // arm can align an arbitrary axis (it cannot).
  Arm a = MakeArm("serial_6dof.urdf", "tool_link");
  CatchPoseIk ik;
  ik.Resize(a.nv);

  const Eigen::VectorXd seed = Eigen::VectorXd::Constant(a.nv, 0.2);
  const Target t = TargetAt(a, seed);
  CatchPoseIkOptions o = BaseOptions();
  o.eps_pos = 1e9;
  o.alpha_max = std::numbers::pi;
  o.max_iter = 1;

  const CatchPoseIkResult r = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, o);
  // Fail-closed: a non-positive pivot is a rejection with its own code, not a
  // w of 0 quietly compared against a threshold, and not `det > 0` deciding on
  // a determinant that rounding could have made either sign.
  EXPECT_EQ(r.reason, CatchPoseReason::kRankDeficient);
  EXPECT_FALSE(r.accepted);
  EXPECT_FALSE(r.w5_valid);
  EXPECT_EQ(r.w5, 0.0);
}

// ── 5. RT contract (SPRINT 6, gate G3-K function part) ───────────────────────

// Positive control for the two gates below. A gate that is inert reports zero
// for everything, which is indistinguishable from success — and this suite's
// whole RT claim rests on those two counters. Both must SEE a deliberate
// allocation before either is allowed to certify its absence.
TEST(CatchPoseIk, TheAllocationGatesAreArmed) {
  std::size_t new_count = 0;
  std::uint64_t eigen_violations = 0;
  {
    const rtc::testing::ScopedNoMalloc eigen_gate;
    const rtc::testing::ScopedAllocGate heap_gate;
    // A runtime-sized Eigen object: the allocation goes through Eigen's own
    // allocator, which is what the tripwire watches.
    Eigen::VectorXd forced = Eigen::VectorXd::Zero(64);
    forced(0) = 1.0;
    // A plain operator-new, which is what the counting gate watches. Eigen's
    // allocator calls std::malloc directly and never reaches operator new, so
    // neither gate can stand in for the other.
    //
    // Via std::vector, not `new double` + `delete`: the standard lets a
    // compiler elide a matched new/delete pair outright ([expr.new]/10), and it
    // does — a control that the optimiser can delete is not a control.
    std::vector<double> owned;
    owned.reserve(32);
    owned.push_back(forced(0));
    new_count = heap_gate.count();
    eigen_violations = eigen_gate.violations();
  }
  EXPECT_GT(new_count, 0U) << "operator-new gate is inert";
  EXPECT_GT(eigen_violations, 0U) << "Eigen allocation tripwire is inert";
}

// Where the RT claim actually has to hold: inside the task QP. CatchPoseIk is
// allocation-free only if QPSolverWrapper::Solve is, and that wrapper's header
// asserts it in prose without a test behind it. Measured here because this is
// the first caller that needs it on a SCHED_FIFO path.
TEST(CatchPoseIk, TheTaskQpItselfAllocatesNothing) {
  constexpr int kN = 6;
  rtc::tsid::QPSolverWrapper qp;
  qp.Init(kN, 0, kN);

  rtc::tsid::QPData data;
  data.Init(kN, 0, kN);
  data.n_vars = kN;
  data.n_eq = 0;
  data.n_ineq = kN;
  data.H = Eigen::MatrixXd::Identity(kN, kN);
  data.C = Eigen::MatrixXd::Identity(kN, kN);
  data.l.setConstant(-0.5);
  data.u.setConstant(0.5);
  data.g << -0.2, 1.1, 0.3, -0.9, 0.4, -1.4;
  ASSERT_TRUE(qp.Solve(data).converged);  // warm the workspace outside the gate

  std::size_t new_count = 0;
  std::uint64_t eigen_violations = 0;
  {
    const rtc::testing::ScopedNoMalloc eigen_gate;
    const rtc::testing::ScopedAllocGate heap_gate;
    qp.ResetWarmStart();
    const bool ok = qp.Solve(data).converged;
    new_count = heap_gate.count();
    eigen_violations = eigen_gate.violations();
    EXPECT_TRUE(ok);
  }
  EXPECT_EQ(new_count, 0U);
  EXPECT_EQ(eigen_violations, 0U);
}

TEST(CatchPoseIk, SolveAllocatesNothingAfterResize) {
  Arm a = Arm6R();
  CatchPoseIk ik;
  ik.Resize(a.nv);
  std::mt19937 rng(333U);

  const Eigen::VectorXd seed = SampleQ(a, rng, 0.4);
  const Target t = TargetAt(a, seed);
  CatchPoseIkOptions o = BaseOptions();
  o.max_iter = 40;
  o.dq_step_max = 0.02;
  o.k_manip = 0.05;  // the finite-difference gradient runs inside the gate too
  o.manip_grad_tol = 1e-5;

  // Warm the model's own lazily-sized internals outside the gate; the contract
  // under test is Solve()'s, not the handle's first-call behaviour.
  (void)ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, o);

  // Both sensors, because neither sees the other's allocations: the gate counts
  // global operator new (a std::vector, a header-inline helper), the scope
  // catches Eigen's allocator, which calls std::malloc directly.
  std::size_t new_count = 0;
  std::uint64_t eigen_violations = 0;
  CatchPoseIkResult r;
  {
    const rtc::testing::ScopedNoMalloc eigen_gate;
    const rtc::testing::ScopedAllocGate heap_gate;
    r = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, o);
    new_count = heap_gate.count();
    eigen_violations = eigen_gate.violations();
  }
  EXPECT_EQ(new_count, 0U);
  EXPECT_EQ(eigen_violations, 0U);
  EXPECT_EQ(r.reason, CatchPoseReason::kNone);
}

TEST(CatchPoseIk, RejectionPathsAlsoAllocateNothing) {
  Arm a = Arm6R();
  CatchPoseIk ik;
  ik.Resize(a.nv);
  const Eigen::VectorXd seed = Eigen::VectorXd::Constant(a.nv, 0.1);
  const CatchPoseIkOptions o = BaseOptions();
  (void)ik.Solve(*a.handle, a.frame, Eigen::Vector3d(50.0, 50.0, 50.0),
                 Eigen::Vector3d(0.0, 0.0, -7.0), seed, o);

  std::size_t new_count = 0;
  std::uint64_t eigen_violations = 0;
  {
    const rtc::testing::ScopedNoMalloc eigen_gate;
    const rtc::testing::ScopedAllocGate heap_gate;
    // The longest rejection path: max_iter iterations that never accept.
    (void)ik.Solve(*a.handle, a.frame, Eigen::Vector3d(50.0, 50.0, 50.0),
                   Eigen::Vector3d(0.0, 0.0, -7.0), seed, o);
    new_count = heap_gate.count();
    eigen_violations = eigen_gate.violations();
  }
  EXPECT_EQ(new_count, 0U);
  EXPECT_EQ(eigen_violations, 0U);
}

// ── 6. Diagnostics that must not lie about what happened (PR #552 review) ────
// Every test below fixes a case where the result POD stayed finite, plausible
// and WRONG about its own run. None of them is a crash or a NaN, which is why
// the sections above could all pass while these failed: a diagnostic that
// reports the opposite of what happened is indistinguishable from a correct one
// until something asks it the question directly.

namespace {

/// σ_min(W J₅) at `q`, read off the fixture's reference Jacobian.
///
/// Shares DifferentialIk with the implementation, deliberately: what is under
/// test here is WHICH configuration the reported number belongs to, not how
/// σ_min is computed (the suite's other oracles cover that).
[[nodiscard]] double SigmaMinOfWeightedTask(Arm& a, const Eigen::VectorXd& q,
                                            const CatchPoseIkOptions& o) {
  const Eigen::MatrixXd j = StackJacobianRef(a, q);
  Eigen::MatrixXd j5w(5, a.nv);
  j5w.topRows(3) = j.topRows(3);
  j5w.row(3) = o.rho * j.row(3);
  j5w.row(4) = o.rho * j.row(4);
  rtc::compliance::DifferentialIk dls;
  dls.Resize(a.nv, 5);
  return dls.Compute(j5w, o.sigma0, o.lambda_max).sigma_min;
}

}  // namespace

TEST(CatchPoseIk, ADeviceOrderedHandleIsRejectedRatherThanSilentlySolved) {
  Arm a = Arm6R();
  CatchPoseIk ik;
  ik.Resize(a.nv);
  std::mt19937 rng(552U);
  const Eigen::VectorXd seed = SampleQ(a, rng, 0.4);
  const Target t = TargetAt(a, seed);
  const CatchPoseIkOptions o = BaseOptions();

  // Positive control: in model order this exact problem is accepted, so the
  // rejection below is caused by the reorder and by nothing else.
  ASSERT_EQ(ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, o).reason, CatchPoseReason::kNone);

  std::vector<std::string> names = a.handle->GetPinocchioJointNames();
  ASSERT_EQ(static_cast<int>(names.size()), a.nv);
  std::reverse(names.begin(), names.end());
  ASSERT_TRUE(a.handle->SetJointOrder(names));
  ASSERT_TRUE(a.handle->HasJointReorder())
      << "the fixture must actually install a permutation, not an identity";

  // The failure being prevented is not a crash. With the permutation active the
  // solve stays finite and still converges — to a pose of a different arm,
  // because ComputeJacobians reads q in device order while the columns it
  // produces, the limits and q̇ are all in Pinocchio order. Only a refusal tells
  // that apart from an answer.
  const CatchPoseIkResult r = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, o);
  EXPECT_EQ(r.reason, CatchPoseReason::kJointOrderMismatch);
  EXPECT_FALSE(r.accepted);
}

TEST(CatchPoseIk, AnUnusableGradientProbeIsNotReportedAsAConvergedAscent) {
  // serial_6dof's J₅ is rank deficient at EVERY configuration, so every
  // central-difference probe of log w₅ comes back invalid: the one fixture on
  // which the ascent can never measure anything at all.
  Arm a = MakeArm("serial_6dof.urdf", "tool_link");
  CatchPoseIk ik;
  ik.Resize(a.nv);

  const Eigen::VectorXd seed = Eigen::VectorXd::Constant(a.nv, 0.2);
  const Target t = TargetAt(a, seed);
  CatchPoseIkOptions o = BaseOptions();
  o.eps_pos = 1e9;  // acceptance vacuous, so the run reaches the ascent at all
  o.alpha_max = std::numbers::pi;
  o.max_iter = 4;
  o.k_manip = 1e-3;
  o.manip_grad_tol = 1e-4;  // a tolerance that ‖0‖ would satisfy

  const CatchPoseIkResult r = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, o);
  // A failed probe leaves grad_norm at 0 because nothing was measured. Reading
  // that as ‖N∇log w₅‖ < tol would break the loop on the first iteration and
  // report manip_converged on a pose whose conditioning was never touched —
  // the exact opposite of what G3-G is supposed to measure.
  EXPECT_FALSE(r.manip_converged);
  EXPECT_EQ(r.manip_grad_failures, o.max_iter);
  EXPECT_EQ(r.iterations, o.max_iter);
  EXPECT_EQ(r.manip_grad_norm, 0.0);
}

TEST(CatchPoseIk, AQpFailureAfterAcceptanceKeepsTheAcceptedPose) {
  Arm a = Arm6R();
  CatchPoseIk ik;
  ik.Resize(a.nv);
  std::mt19937 rng(553U);
  const Eigen::VectorXd seed = SampleQ(a, rng, 0.4);
  const Target t = TargetAt(a, seed);

  // A QP that cannot converge: one iteration against a 1e-14 tolerance. The
  // target is offset by 1 mm so the residual — and therefore g — is nonzero,
  // since at g = 0 the origin is optimal and even one iteration succeeds.
  CatchPoseIkOptions o = BaseOptions();
  o.eps_pos = 5e-3;  // …but well inside the tolerance, so iteration 1 accepts
  o.alpha_max = 1e-2;
  o.k_manip = 1e-6;        // the ascent is what keeps the loop running…
  o.manip_grad_tol = 0.0;  // …past the iterate that already met the task
  o.max_iter = 8;
  o.qp_max_iter = 1;
  o.qp_eps_abs = 1e-14;
  const Eigen::Vector3d p_near = t.p_c + Eigen::Vector3d(1e-3, 0.0, 0.0);

  const CatchPoseIkResult r = ik.Solve(*a.handle, a.frame, p_near, t.v_ball, seed, o);
  ASSERT_EQ(r.qp_failures, 1) << "the fixture must actually provoke a QP failure";
  // The pose was already accepted under the same law, and a later QP failure
  // only means the run cannot keep improving it. Discarding it would turn a
  // valid catch pose into a rejection carrying an all-zero q.
  EXPECT_EQ(r.reason, CatchPoseReason::kNone);
  EXPECT_TRUE(r.accepted);
  EXPECT_GT(ResultQ(r).lpNorm<Eigen::Infinity>(), 0.0);
  EXPECT_LT(r.pos_error, o.eps_pos);
  EXPECT_LE(r.theta, o.alpha_max);

  // The other half of fail-closed is unchanged: with nothing accepted yet there
  // is no pose to keep, and the same failing QP is a rejection.
  CatchPoseIkOptions strict = o;
  strict.eps_pos = 1e-9;
  const CatchPoseIkResult rejected =
      ik.Solve(*a.handle, a.frame, Eigen::Vector3d(50.0, 50.0, 50.0), t.v_ball, seed, strict);
  EXPECT_EQ(rejected.reason, CatchPoseReason::kQpFailed);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.qp_failures, 1);
}

TEST(CatchPoseIk, TheProjectorDiagnosticsDescribeQStarNotTheLastIterate) {
  Arm a = Arm6R();
  CatchPoseIk ik;
  ik.Resize(a.nv);
  std::mt19937 rng(554U);
  const Eigen::VectorXd seed = SampleQ(a, rng, 0.4);
  const Target t = TargetAt(a, seed);

  // Iteration 1 sits exactly on the target and is accepted; a deliberately
  // large ascent gain then throws iterate 2 far outside eps_pos. The run
  // therefore ENDS on an iterate that is not q*, which is the only situation in
  // which "at q*" and "at the last iterate" are different claims.
  CatchPoseIkOptions tight = BaseOptions();
  tight.max_iter = 2;
  tight.k_manip = 5.0;
  tight.manip_grad_tol = 0.0;  // never stop early — both runs take both steps

  // The same trajectory read differently: acceptance made vacuous, so q* IS the
  // last iterate. The update law does not depend on acceptance, so the two runs
  // visit the same two configurations.
  CatchPoseIkOptions vacuous = tight;
  vacuous.eps_pos = 1e9;
  vacuous.alpha_max = std::numbers::pi;

  const CatchPoseIkResult acc = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, tight);
  const CatchPoseIkResult last = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, seed, vacuous);
  ASSERT_TRUE(acc.accepted);
  ASSERT_TRUE(last.accepted);
  ASSERT_LT((ResultQ(acc) - seed).lpNorm<Eigen::Infinity>(), 1e-12);
  ASSERT_GT((ResultQ(acc) - ResultQ(last)).lpNorm<Eigen::Infinity>(), 1e-3)
      << "the fixture must make q* and the last iterate different poses";
  ASSERT_GT(std::abs(acc.sigma_min - last.sigma_min), 1e-6)
      << "…and must make their σ_min tell those poses apart";

  EXPECT_NEAR(acc.sigma_min, SigmaMinOfWeightedTask(a, ResultQ(acc), tight), 1e-9);
  EXPECT_NEAR(last.sigma_min, SigmaMinOfWeightedTask(a, ResultQ(last), vacuous), 1e-9);
}

TEST(CatchPoseIk, AnOutOfLimitSeedIsNotAPermanentPosturePull) {
  Arm a = Arm6R();
  CatchPoseIk ik;
  ik.Resize(a.nv);
  std::mt19937 rng(555U);

  Eigen::VectorXd raw = SampleQ(a, rng, 0.4);
  raw(1) = a.model->upperPositionLimit(1) + 0.3;  // one joint past its bound
  Eigen::VectorXd clamped = raw;
  clamped(1) = a.model->upperPositionLimit(1);

  // A target a short way off the clamped seed: far enough that the run takes
  // real iterations with the posture term active, close enough that a k_null
  // that fights the task cannot turn the test into a kNotConverged.
  Eigen::VectorXd goal = clamped;
  goal(3) += 0.05;
  goal(5) -= 0.05;
  const Target t = TargetAt(a, goal);
  CatchPoseIkOptions o = BaseOptions();
  // k_null and eps_pos are chosen together, and the `control` run below is what
  // proves the pair is usable. N makes q̇_n invisible to the task only to FIRST
  // order, so a posture step of ~k_null·‖q_n − q‖ leaves a second-order
  // residual the next task step has to undo; push k_null up and the loop
  // settles into a standing error instead of converging. That is a property of
  // the law, not of this fixture. 2e-3 is the eps_pos the header ships.
  o.k_null = 0.1;
  o.eps_pos = 2e-3;

  // The raw seed is used for exactly two things — the first iterate and the
  // posture reference q_n — and both are the CLAMPED vector, so the two calls
  // are the same input and must give the same answer bit for bit. With q_n left
  // raw they are not: k_null·(q_n − q) can never reach a target outside the
  // limits, so it never decays, and N spreads that permanent pull onto every
  // joint while the clamp keeps undoing it on the one that is out of bounds.
  const CatchPoseIkResult from_raw = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, raw, o);
  const CatchPoseIkResult from_clamped = ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, clamped, o);

  CatchPoseIkOptions no_posture = o;
  no_posture.k_null = 0.0;
  const CatchPoseIkResult control =
      ik.Solve(*a.handle, a.frame, t.p_c, t.v_ball, clamped, no_posture);
  ASSERT_EQ(control.reason, CatchPoseReason::kNone)
      << "control reason " << static_cast<int>(control.reason) << ", iters " << control.iterations;
  ASSERT_EQ(from_clamped.reason, CatchPoseReason::kNone)
      << "reason " << static_cast<int>(from_clamped.reason) << ", iters " << from_clamped.iterations
      << ", pos_error " << from_clamped.pos_error << ", theta " << from_clamped.theta;
  ASSERT_GT(from_clamped.iterations, 1)
      << "the fixture must run long enough for a posture term to act";
  ASSERT_GT((ResultQ(from_clamped) - clamped).lpNorm<Eigen::Infinity>(), 1e-3)
      << "…and must actually move away from the seed";

  EXPECT_EQ(from_raw.reason, from_clamped.reason);
  EXPECT_EQ(from_raw.iterations, from_clamped.iterations);
  for (int i = 0; i < a.nv; ++i)
    EXPECT_EQ(from_raw.q[static_cast<std::size_t>(i)], from_clamped.q[static_cast<std::size_t>(i)])
        << "joint " << i;
}
