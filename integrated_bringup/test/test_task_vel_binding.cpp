// ── Task-velocity assembly: demo_task_controller's inline form → task_vel_law ─
// Issue #314, the second and last half of the #282 convergence (the first was
// the DLS, in test_task_dls_convergence.cpp). Unlike #282 this one is NOT a law
// change: both sides compute kp ⊙ e + ν_ff with the same operands in the same
// order, so the claim available here is the strong one — bit-for-bit inert — and
// a tolerance would be the wrong sensor. What moved (compute.cpp, both
// branches):
//
//   6-DOF:  kp_vec_6d built inline, then kp_vec_6d.cwiseProduct(e) + ff_vel_6d
//           → rtc::task::ComputeTaskVelocity(TaskVelParams{kp_t, kp_r}, e, ff)
//   3-DOF:  kp_vec built inline, then kp_vec.cwiseProduct(e) + ff_lin
//           → rtc::task::ComputeTranslationVelocity(kp_t, e, ff_lin)
//
// ── Why this is a NEW oracle and not a copy of an existing one ───────────────
// rtc_controllers/test/test_task_vel_core.cpp already pins the same two core
// functions bitwise — but against ClikController's pre-extraction shape, where
// the rotation product lands DIRECTLY in the accumulation:
//
//     task_vel = kp.cwiseProduct(e);           // materialise
//     task_vel.head<3>() += R_traj * nu_lin;   // product inside the +=
//
// demo_task_controller's shape is a different one: the feedforward is rotated
// into a NAMED TEMPORARY first (it has to be — the same ff_vel_6d is reused for
// the log-only traj_dq_ lane), and the sum is then a SINGLE expression:
//
//     ff_vel_6d.head<3>() = R * nu_lin;        // materialised first
//     task_vel_6d = kp.cwiseProduct(e) + ff_vel_6d;   // one expression
//
// against a core that spells the same thing as TWO statements (materialise,
// then +=). task_vel_law.hpp says it outright — "the two forms differ in
// expression shape, not just in size, and the bitwise claim is per-shape" — so
// the shape this binding actually ships is pinned here or it is pinned nowhere.
//
// ── What makes the claim true, and the flag it depends on ───────────────────
// Measured before the refactor was written, over 600k draws per regime: the two
// shapes agree on every bit under the flags this repo builds with (-O3, baseline
// x86-64, no -march). They do NOT agree if FMA is available: with -march=native
// or -mfma the single-expression form contracts to vfmadd and the two-statement
// form does not, and the lanes separate by ~1e-14. So the inertness is real for
// what ships, and a build that enables FMA would fail these tests rather than
// silently drift — which is the correct failure, because it is exactly the
// build under which the pre-#314 and post-#314 binaries would have differed.
// The same conditionality already applies to the #236 bitwise suites; #314 does
// not introduce it.
#include "rtc_controllers/task/task_vel_law.hpp"
#include "rtc_controllers/testing/bit_compare.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <random>

namespace {

using rtc::task::ComputeTaskVelocity;
using rtc::task::ComputeTranslationVelocity;
using rtc::task::TaskVelParams;
using rtc::testing::BitsEqual;
using Vec6 = Eigen::Matrix<double, 6, 1>;

// One tick's worth of inputs, in the shape the BINDING holds them: the
// feedforward is already rotated into the Jacobian frame (that rotation stays on
// the binding's side of the contract and is not what this test compares).
struct CoreDraw {
  std::array<double, 3> kp_pos{};
  std::array<double, 3> kp_rot{};
  Vec6 pos_error{Vec6::Zero()};
  Vec6 ff_vel{Vec6::Zero()};
};

// EXACTLY the expression as it stood inline in DemoTaskController's task
// ComputeControl before #314 routed it through task/task_vel_law.hpp
// (integrated_bringup/src/controllers/task/compute.cpp:506-527 at 136708bf).
// Rename-only transcription:
//   gains.kp_translation → d.kp_pos    gains.kp_rotation → d.kp_rot
//   pos_error_6d_        → d.pos_error ff_vel_6d         → d.ff_vel
// Nothing else moved: the gain vector is built by the same loop in the same
// order (translation 0..2, then rotation 3..5), and the sum stays a single
// expression over an already-materialised feedforward.
Vec6 PreConvergenceInlineForm6(const CoreDraw& d) {
  Vec6 kp_vec_6d;
  for (std::size_t i = 0; i < 3; ++i) {
    kp_vec_6d[static_cast<Eigen::Index>(i)] = d.kp_pos[i];
    kp_vec_6d[static_cast<Eigen::Index>(i + 3)] = d.kp_rot[i];
  }
  const Vec6 task_vel_6d = kp_vec_6d.cwiseProduct(d.pos_error) + d.ff_vel;
  return task_vel_6d;
}

// Likewise for the translation-only branch (compute.cpp:535-546 at 136708bf).
Eigen::Vector3d PreConvergenceInlineForm3(const CoreDraw& d) {
  Eigen::Vector3d kp_vec(d.kp_pos[0], d.kp_pos[1], d.kp_pos[2]);
  const Eigen::Vector3d task_vel = kp_vec.cwiseProduct(Eigen::Vector3d(d.pos_error.head<3>())) +
                                   Eigen::Vector3d(d.ff_vel.head<3>());
  return task_vel;
}

// Drive the core with the same draw — this is the marshalling compute.cpp now
// does.
Vec6 RunCore6(const CoreDraw& d) {
  return ComputeTaskVelocity(TaskVelParams{d.kp_pos, d.kp_rot}, d.pos_error, d.ff_vel);
}

Eigen::Vector3d RunCore3(const CoreDraw& d) {
  return ComputeTranslationVelocity(d.kp_pos, Eigen::Vector3d(d.pos_error.head<3>()),
                                    Eigen::Vector3d(d.ff_vel.head<3>()));
}

std::mt19937 MakeRng() {
  return std::mt19937(20260729U);
}

// Draws sweep the magnitudes this binding actually meets: kp_translation and
// kp_rotation are velocity-form and O(1)-O(10), the pose error is small once the
// arm tracks and O(0.1) right after a retarget, and the feedforward dominates
// mid-trajectory. kp_pos and kp_rot are drawn INDEPENDENTLY so the gain-vector
// ordering is under test too — if the core assembled [rot, trans] the six lanes
// would disagree. Every few trials a term is zeroed or made near-cancelling: a
// cancellation that only shows up when the two terms are comparable is precisely
// what a regrouping changes, and it is the only regime where the association
// could plausibly matter.
CoreDraw RandomDraw(std::mt19937& rng, int trial) {
  std::uniform_real_distribution<double> gain(0.2, 20.0);
  std::normal_distribution<double> err(0.0, 0.3);
  std::normal_distribution<double> vel(0.0, 0.8);

  CoreDraw d;
  for (std::size_t i = 0; i < 3; ++i) {
    d.kp_pos[i] = gain(rng);
    d.kp_rot[i] = gain(rng);
  }
  for (int i = 0; i < 6; ++i) {
    d.pos_error[i] = err(rng);
    d.ff_vel[i] = vel(rng);
  }

  switch (trial % 5) {
    case 1:
      d.pos_error.setZero();  // converged — the feedforward alone
      break;
    case 2:
      d.ff_vel.setZero();  // holding a target — the proportional term alone
      break;
    case 3:
      // The two terms nearly cancel: the arm is being driven back exactly as
      // fast as the trajectory carries it forward.
      for (int i = 0; i < 6; ++i) {
        const double kp = (i < 3) ? d.kp_pos[static_cast<std::size_t>(i)]
                                  : d.kp_rot[static_cast<std::size_t>(i - 3)];
        d.ff_vel[i] = -kp * d.pos_error[i] * (1.0 + std::ldexp(1.0, -45));
      }
      break;
    default:
      break;
  }
  return d;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Bit-identity against the literal pre-convergence forms
// ═══════════════════════════════════════════════════════════════════════════

TEST(TaskVelBinding, MatchesThePreConvergenceInlineFormBitwise) {
  auto rng = MakeRng();
  double peak = 0.0;
  for (int trial = 0; trial < 600; ++trial) {
    const CoreDraw d = RandomDraw(rng, trial);

    const Vec6 got = RunCore6(d);
    const Vec6 want = PreConvergenceInlineForm6(d);

    for (int i = 0; i < 6; ++i) {
      EXPECT_TRUE(BitsEqual(got[i], want[i]))
          << "trial " << trial << " axis " << i << ": " << got[i] << " vs " << want[i];
      peak = std::max(peak, std::abs(got[i]));
    }
  }
  EXPECT_GT(peak, 1e-6) << "every compared axis was ~0 — the draw generator degenerated";
}

TEST(TaskVelBinding, MatchesThePreConvergenceInlineFormBitwiseTranslationOnly) {
  auto rng = MakeRng();
  double peak = 0.0;
  for (int trial = 0; trial < 600; ++trial) {
    const CoreDraw d = RandomDraw(rng, trial);

    const Eigen::Vector3d got = RunCore3(d);
    const Eigen::Vector3d want = PreConvergenceInlineForm3(d);

    for (int i = 0; i < 3; ++i) {
      EXPECT_TRUE(BitsEqual(got[i], want[i]))
          << "trial " << trial << " axis " << i << ": " << got[i] << " vs " << want[i];
      peak = std::max(peak, std::abs(got[i]));
    }
  }
  EXPECT_GT(peak, 1e-6) << "every compared axis was ~0 — the draw generator degenerated";
}

// Non-vacuity partner. Splitting the feedforward across two additions is exact
// in the scaling (×0.5 costs no bits) and algebraically identical, but it rounds
// twice where the shipped form rounds once — so if bit equality could not tell
// the two apart, the two tests above would be pinning nothing. This is the
// partner test bit_compare.hpp requires of every extraction suite.
TEST(TaskVelBinding, BitwiseComparisonRejectsASplitAccumulation) {
  auto rng = MakeRng();
  int differing_axes = 0;
  int compared = 0;

  for (int trial = 0; trial < 600; ++trial) {
    const CoreDraw d = RandomDraw(rng, trial);
    const Vec6 got = RunCore6(d);

    for (int i = 0; i < 6; ++i) {
      const double kp = (i < 3) ? d.kp_pos[static_cast<std::size_t>(i)]
                                : d.kp_rot[static_cast<std::size_t>(i - 3)];
      const double half = d.ff_vel[i] * 0.5;
      double split = kp * d.pos_error[i] + half;
      split += half;

      ++compared;
      if (!BitsEqual(got[i], split))
        ++differing_axes;
      EXPECT_NEAR(got[i], split, 1e-6 * (1.0 + std::abs(got[i])))
          << "the mutant must stay algebraically equivalent, else it proves nothing";
    }
  }
  ASSERT_GT(compared, 0) << "no axes compared — the draw generator degenerated";
  EXPECT_GT(differing_axes, 0)
      << "bit comparison cannot distinguish a split accumulation — the equivalence tests above "
         "are vacuous";
}

// The gain-vector ORDER is part of what moved: the binding built
// [trans, trans, trans, rot, rot, rot] by hand and the core builds it inside
// ComputeTaskVelocity. A core that assembled the two halves the other way round
// would still be internally consistent, so pin the ordering directly rather than
// relying on the draws to catch it.
TEST(TaskVelBinding, GainVectorOrdersTranslationBeforeRotation) {
  CoreDraw d;
  d.kp_pos = {2.0, 3.0, 5.0};
  d.kp_rot = {7.0, 11.0, 13.0};
  d.pos_error = (Vec6() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0).finished();
  d.ff_vel.setZero();

  const Vec6 got = RunCore6(d);
  const Vec6 want = (Vec6() << 2.0, 3.0, 5.0, 7.0, 11.0, 13.0).finished();
  for (int i = 0; i < 6; ++i) {
    EXPECT_TRUE(BitsEqual(got[i], want[i]))
        << "axis " << i << ": task gains are not [translation×3, rotation×3]";
  }
}
