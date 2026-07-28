// ── Golden-vector harness for the S6 cores (#236 S6) ────────────────────────
// Sixth slice of the rtc_controllers pure-library refactor, and the one whose
// ARCH-3 trigger is exact rather than approximate: the null-space posture loop
// in TaskImpedanceController::Compute() and the one in
// CascadedComplianceController::Compute() were CHARACTER-FOR-CHARACTER the same
// code. Two cores come out of it:
//
//   • joint/posture_law.hpp — ComputePostureTorque (PD, torque domain) and
//     ComputePostureVelocity (P, velocity domain);
//   • compliance/bandwidth_separation.hpp — the §7.6 MUST-1 verdict, previously
//     CascadedComplianceController::EvaluateBandwidthSeparation.
//
// Same contract S1 established and S2a/S3a/S4/S5 followed: the claim under test
// is not "the core is correct" but the stronger "the migration changed
// NOTHING", so every comparison is bitwise (rtc::testing::BitsEqual), never a
// tolerance.
//
// ── Why the posture law is TWO functions ────────────────────────────────────
// TaskAdmittanceController runs the P form, which is the PD form at K_d = 0 —
// algebraically. Not bitwise: `x − 0.0·q̇` is `x` for every finite non-zero x,
// but for x = −0.0 and q̇ < 0 IEEE-754 gives (−0.0) − (−0.0) = +0.0. That state
// is reachable and in fact ordinary — q_ref is seeded FROM the measurement, so
// q_ref − q is exactly +0.0 while the arm holds still, and a negative K_p (which
// neither TaskAdmittanceController::LoadConfig nor
// TaskImpedanceController::LoadConfig floors, and which set_gains() can write
// past any floor) makes x = −0.0 on every channel. The pre-extraction probe
// measured a flip on 50% of draws in that state at -O0/-O2/-O3. So: two shapes,
// two functions, two literal oracles — the call S3a made for task_vel_law.hpp's
// two forms, and TheProportionalFormIsNotThePdFormWithZeroDamping below keeps
// the measurement RESIDENT instead of inheriting it from a session probe.
//
// ── Oracles ─────────────────────────────────────────────────────────────────
//
//   1. PreExtraction*Form — literal, rename-only copies of the three inline
//      loops and of the removed member function, at 8526a4c9. DURABLE: they
//      outlive the adapters, so they still pin the cores after S7 deletes them.
//
//   2. The live CascadedComplianceController. TRANSIENT (dies with S7). Its job
//      is what oracle 1 structurally cannot do: catch a CORRELATED transcription
//      error, and pin the WIRING — that q_null/q_dev/q̇_dev reach the law in that
//      order and that role, that the gains marshalled in are nullspace_kp/_kd,
//      that Λ_S reaches the verdict as the matrix whose diagonal it reads, and
//      that both gates still open and close where they did.
//
// ── What the gates force this file to do (S4 (0b), S5 M4) ───────────────────
// Both cores sit behind a gate, and BOTH gates are numerically inert when
// closed, so a closed-gate case alone proves nothing:
//
//   • the posture gate `(nv > 6) && (kp != 0 || kd != 0)` — with the gains zero
//     the ungated path would contribute `0.0·Δq − 0.0·q̇`, signed zeros that Nᵀ
//     projects and the torque sum absorbs exactly. S5 measured this: removing
//     the gate from the adapter left every bitwise lane green. The only window
//     is diag.nullspace_active, so this suite compares it EVERY TICK.
//   • the `just_seeded || bw_pending` gate is worse — skipping the evaluation
//     leaves the PREVIOUS value in place, so the diagnostic does not move at
//     all. Hence AdapterReEvaluatesTheRatioWhenTheGainsChange (the set_gains
//     path) and AdapterRetriesAfterAFailedEvaluation (the Cholesky path), which
//     are the only two ways that gate reopens.

// no_malloc_scope.hpp MUST precede every Eigen header — it installs the Eigen
// allocation tripwire by defining EIGEN_RUNTIME_NO_MALLOC and its own
// eigen_assert, and a later include would be silently ignored (its own #error
// enforces the ordering). Everything below pulls Eigen, so it comes first.
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_controllers/compliance/admittance_integrator.hpp"
#include "rtc_controllers/compliance/bandwidth_separation.hpp"
#include "rtc_controllers/compliance/impedance_law.hpp"
#include "rtc_controllers/compliance/task_dynamics.hpp"
#include "rtc_controllers/direct/cascaded_compliance_controller.hpp"
#include "rtc_controllers/joint/posture_law.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/bit_compare.hpp"
#include "rtc_controllers/testing/serial7dof_fixture.hpp"
#include "rtc_math/se3/pinocchio_adapter.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <random>
#include <span>
#include <string>
#include <vector>

namespace {

using Vec6 = Eigen::Matrix<double, 6, 1>;

using rtc::CascadedComplianceController;
using rtc::kMaxDeviceChannels;
using rtc::compliance::AdmittanceParams;
using rtc::compliance::BandwidthSeparation;
using rtc::compliance::EvaluateBandwidthSeparation;
using rtc::compliance::ImpedanceParams;
using rtc::joint::ComputePostureTorque;
using rtc::joint::ComputePostureVelocity;
using rtc::joint::PostureInputs;
using rtc::testing::BitsEqual;
using rtc::testing::FillSweep;
using rtc::testing::MakeHandle;
using rtc::testing::MakeRng;
using rtc::testing::MakeState;
using rtc::testing::Serial7dof;

constexpr double kDt = 0.002;
constexpr int kNv = 7;

// Mirrors the adapter's own anonymous-namespace constant (NUM-1 floor on λ_max,
// cascaded_compliance_controller.cpp). Copied rather than exported because the
// floor is the ADAPTER's decision about its own YAML surface, not part of a law
// under test — the same call S4's and S5's shims made.
constexpr double kMinMaxDamping = 1e-4;

// ═══════════════════════════════════════════════════════════════════════════
// Oracle 1a — the posture law, literal pre-extraction forms
// ═══════════════════════════════════════════════════════════════════════════

// One random tick's worth of posture inputs, in the shape the ADAPTERS hold
// them: Eigen vectors in DEVICE channel order, and two scalar gains.
struct PostureDraw {
  double kp{0.0};
  double kd{0.0};
  Eigen::VectorXd q_ref{Eigen::VectorXd::Zero(kNv)};
  Eigen::VectorXd q{Eigen::VectorXd::Zero(kNv)};
  Eigen::VectorXd qdot{Eigen::VectorXd::Zero(kNv)};
};

// EXACTLY the loop as it stood inline in TaskImpedanceController::Compute() and
// — character for character — in CascadedComplianceController::Compute() before
// #236 S6 routed them through joint/posture_law.hpp
// (task_impedance_controller.cpp:472-474 and
// cascaded_compliance_controller.cpp:540-542 at 8526a4c9). Rename-only:
//   gains.nullspace_kp → d.kp   q_null_ → d.q_ref
//   gains.nullspace_kd → d.kd   q_dev_  → d.q      qdot_dev_ → d.qdot
Eigen::VectorXd PreExtractionTorqueForm(const PostureDraw& d, int nv) {
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(nv);
  for (int i = 0; i < nv; ++i)
    tau(i) = d.kp * (d.q_ref(i) - d.q(i)) - d.kd * d.qdot(i);
  return tau;
}

// EXACTLY the loop as it stood inline in TaskAdmittanceController::Compute()
// (task_admittance_controller.cpp:386-388 at 8526a4c9). The P form: no velocity
// term at all, not a zeroed one.
Eigen::VectorXd PreExtractionVelocityForm(const PostureDraw& d, int nv) {
  Eigen::VectorXd v = Eigen::VectorXd::Zero(nv);
  for (int i = 0; i < nv; ++i)
    v(i) = d.kp * (d.q_ref(i) - d.q(i));
  return v;
}

PostureInputs InputsOf(const PostureDraw& d, std::size_t n) {
  return PostureInputs{{d.q_ref.data(), n}, {d.q.data(), n}, {d.qdot.data(), n}};
}

Eigen::VectorXd RunCoreTorque(const PostureDraw& d, int nv) {
  const auto n = static_cast<std::size_t>(nv);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(nv);
  ComputePostureTorque(d.kp, d.kd, InputsOf(d, n), n, {tau.data(), n});
  return tau;
}

Eigen::VectorXd RunCoreVelocity(const PostureDraw& d, int nv) {
  const auto n = static_cast<std::size_t>(nv);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(nv);
  ComputePostureVelocity(d.kp, InputsOf(d, n), n, {v.data(), n});
  return v;
}

// Draws sweep the states this law actually meets. The posture ERROR is small by
// construction — q_ref is the seed, so Δq is whatever the arm has drifted since
// activation — and the velocity is a joint rate. Every fifth trial forces one of
// the states the law is defined at but a uniform draw would never produce:
// the exact seed (Δq bit-exactly +0.0, the state that makes the K_d = 0
// reduction unsafe), a retired stiffness, a retired damping, and a
// near-cancelling combination where the P and D terms nearly annihilate — the
// only regime where a regrouping of the two terms could change a bit.
PostureDraw RandomDraw(std::mt19937& rng, int trial) {
  std::uniform_real_distribution<double> stiffness(0.2, 40.0);
  std::uniform_real_distribution<double> damping(0.05, 6.0);
  std::uniform_real_distribution<double> angle(-2.8, 2.8);
  std::normal_distribution<double> drift(0.0, 0.15);
  std::normal_distribution<double> rate(0.0, 0.4);

  PostureDraw d;
  d.kp = stiffness(rng);
  d.kd = damping(rng);
  for (int i = 0; i < kNv; ++i) {
    d.q(i) = angle(rng);
    d.q_ref(i) = d.q(i) + drift(rng);
    d.qdot(i) = rate(rng);
  }

  switch (trial % 5) {
    case 1:
      d.q_ref = d.q;  // exactly at the seed: Δq is +0.0 on every channel
      break;
    case 2:
      d.kp = 0.0;  // damping only — the gate's second disjunct
      break;
    case 3:
      d.kd = 0.0;  // stiffness only
      break;
    case 4:
      // Near-cancelling: the damping term almost exactly opposes the P term, so
      // the difference is a catastrophic subtraction on every channel.
      for (int i = 0; i < kNv; ++i)
        d.qdot(i) = d.kp * (d.q_ref(i) - d.q(i)) / d.kd;
      break;
    default:
      break;
  }
  return d;
}

struct DrawCounts {
  int at_seed{0};
  int zero_kp{0};
  int zero_kd{0};
  int both_gains{0};
};

void Tally(const PostureDraw& d, DrawCounts* c) {
  if (d.q_ref == d.q)
    ++c->at_seed;
  if (d.kp == 0.0)
    ++c->zero_kp;
  if (d.kd == 0.0)
    ++c->zero_kd;
  if (d.kp != 0.0 && d.kd != 0.0)
    ++c->both_gains;
}

}  // namespace

TEST(PostureLaw, MatchesThePreExtractionInlineFormBitwiseTorque) {
  auto rng = MakeRng();
  double peak = 0.0;
  DrawCounts counts;

  for (int trial = 0; trial < 600; ++trial) {
    const PostureDraw d = RandomDraw(rng, trial);
    Tally(d, &counts);

    const Eigen::VectorXd got = RunCoreTorque(d, kNv);
    const Eigen::VectorXd want = PreExtractionTorqueForm(d, kNv);

    for (int i = 0; i < kNv; ++i) {
      EXPECT_TRUE(BitsEqual(got(i), want(i)))
          << "trial " << trial << " channel " << i << ": " << got(i) << " vs " << want(i);
      peak = std::max(peak, std::abs(got(i)));
    }
  }

  EXPECT_GT(peak, 1e-6) << "every compared channel was ~0 — the draw generator degenerated";
  // The states the draw is supposed to reach. Without these the suite could be
  // pinning one regime and calling it the law — and the seed state in particular
  // is the one the P-form separation exists for.
  EXPECT_GT(counts.at_seed, 0) << "no draw sat exactly at the posture seed (Δq = +0.0)";
  EXPECT_GT(counts.zero_kp, 0) << "no draw retired the stiffness (the gate's kd-only disjunct)";
  EXPECT_GT(counts.zero_kd, 0) << "no draw retired the damping";
  EXPECT_GT(counts.both_gains, 0) << "no draw ran the full PD form";
}

TEST(PostureLaw, MatchesThePreExtractionInlineFormBitwiseVelocity) {
  auto rng = MakeRng();
  double peak = 0.0;
  DrawCounts counts;

  for (int trial = 0; trial < 600; ++trial) {
    const PostureDraw d = RandomDraw(rng, trial);
    Tally(d, &counts);

    const Eigen::VectorXd got = RunCoreVelocity(d, kNv);
    const Eigen::VectorXd want = PreExtractionVelocityForm(d, kNv);

    for (int i = 0; i < kNv; ++i) {
      EXPECT_TRUE(BitsEqual(got(i), want(i)))
          << "trial " << trial << " channel " << i << ": " << got(i) << " vs " << want(i);
      peak = std::max(peak, std::abs(got(i)));
    }
  }

  EXPECT_GT(peak, 1e-6) << "every compared channel was ~0 — the draw generator degenerated";
  EXPECT_GT(counts.at_seed, 0) << "no draw sat exactly at the posture seed (Δq = +0.0)";
}

// Non-vacuity partner, and NOT a different spelling of the same algorithm — the
// S4 probe's lesson (an out-of-place solve turned out bit-identical to
// solveInPlace, so a mutant built that way would have proved nothing). This one
// contracts the two terms with std::fma, computing kp·Δq − kd·q̇ in ONE rounding
// instead of two: algebraically the same value, a genuinely different algorithm.
// If bit equality could not tell these apart, the test above would be pinning
// nothing — in particular it could not distinguish the extraction it exists to
// certify from a compiler that fused the expression.
TEST(PostureLaw, BitwiseComparisonRejectsAFusedMultiplyAdd) {
  auto rng = MakeRng();
  int differing = 0;
  int compared = 0;

  for (int trial = 0; trial < 600; ++trial) {
    const PostureDraw d = RandomDraw(rng, trial);
    const Eigen::VectorXd got = RunCoreTorque(d, kNv);

    for (int i = 0; i < kNv; ++i) {
      const double fused = std::fma(-d.kd, d.qdot(i), d.kp * (d.q_ref(i) - d.q(i)));
      ++compared;
      if (!BitsEqual(got(i), fused))
        ++differing;
      const double scale = 1.0 + std::abs(got(i)) + std::abs(fused);
      EXPECT_NEAR(got(i), fused, 1e-9 * scale)
          << "the mutant must stay algebraically equivalent, else it proves nothing";
    }
  }

  ASSERT_GT(compared, 0) << "no channels compared — the draw generator degenerated";
  EXPECT_GT(differing, 0)
      << "bit comparison cannot distinguish a fused multiply-add — the equivalence test is vacuous";
}

// The resident form of the pre-extraction probe that split the law in two. It is
// a test and not a comment because the alternative — one function, called with
// K_d = 0 from the admittance binding — is the obvious "simplification" a later
// reader will reach for, and the reason it is wrong is a signed zero that no
// tolerance-based test and no ordinary draw can see.
//
// The reachable state, spelled out: q_ref is seeded FROM the measurement
// (task_admittance_controller.cpp assigns q_null_(i) = q_dev_(i)), so Δq is
// exactly +0.0 while the arm holds still; a negative K_p — which that
// controller's LoadConfig accepts, since it floors nothing — turns kp·(+0.0)
// into −0.0; and a negative joint velocity makes 0.0·q̇ = −0.0, so the PD form
// evaluates (−0.0) − (−0.0) = +0.0 where the P form keeps −0.0.
TEST(PostureLaw, TheProportionalFormIsNotThePdFormWithZeroDamping) {
  PostureDraw d;
  d.kp = -3.0;  // reachable: neither admittance nor impedance LoadConfig floors it
  d.kd = 0.0;
  for (int i = 0; i < kNv; ++i) {
    d.q(i) = 0.31 * (1.0 + i);
    d.q_ref(i) = d.q(i);    // holding at the seed
    d.qdot(i) = -0.05 - i;  // moving negative on every channel
  }

  const Eigen::VectorXd p_form = RunCoreVelocity(d, kNv);
  const Eigen::VectorXd pd_form = RunCoreTorque(d, kNv);

  int flipped = 0;
  for (int i = 0; i < kNv; ++i) {
    // Numerically indistinguishable — this is a signed zero, not a wrong answer.
    EXPECT_DOUBLE_EQ(p_form(i), pd_form(i)) << "channel " << i;
    EXPECT_EQ(p_form(i), 0.0) << "channel " << i << ": the state under test is a signed zero";
    if (!BitsEqual(p_form(i), pd_form(i)))
      ++flipped;
  }
  EXPECT_EQ(flipped, kNv) << "the K_d = 0 reduction was bitwise inert on a state where the "
                             "pre-extraction probe measured it flipping — if this is genuinely "
                             "no longer reachable, retire ComputePostureVelocity rather than "
                             "weakening this expectation";
  EXPECT_TRUE(std::signbit(p_form(0))) << "the P form must keep the negative zero";
  EXPECT_FALSE(std::signbit(pd_form(0))) << "the PD form must produce the positive zero";

  // Narrowness: away from that state the two forms DO agree bitwise, so the
  // separation is about the signed zero and not about the law having changed.
  auto rng = MakeRng();
  int agreeing = 0;
  int compared = 0;
  for (int trial = 0; trial < 200; ++trial) {
    PostureDraw g = RandomDraw(rng, trial);
    g.kd = 0.0;
    if (g.kp == 0.0 || g.q_ref == g.q)
      continue;  // the very states the flip needs — excluded on purpose
    const Eigen::VectorXd p = RunCoreVelocity(g, kNv);
    const Eigen::VectorXd pd = RunCoreTorque(g, kNv);
    for (int i = 0; i < kNv; ++i) {
      ++compared;
      if (BitsEqual(p(i), pd(i)))
        ++agreeing;
    }
  }
  ASSERT_GT(compared, 0) << "the narrowness half compared nothing";
  EXPECT_EQ(agreeing, compared)
      << "the two forms diverge away from the signed-zero state too — then the split is not "
         "about zero signs and this test is describing the wrong defect";
}

// Eigen::Ref-style size confusion has already shipped once in this refactor (S4:
// a fixed 6×6 read past a 3×3 Λ_S because the Ref carried its own size and
// eigen_assert is compiled out of the library TU). The span-based law bounds by
// every span it reads, and the tail is ZEROED rather than left alone because the
// buffer is about to be projected and added to a torque — a stale value there is
// a torque, a fresh zero is passive.
TEST(PostureLaw, ShortInputsDegradeToZeroInsteadOfReadingPastThem) {
  PostureDraw d;
  d.kp = 5.0;
  d.kd = 0.5;
  for (int i = 0; i < kNv; ++i) {
    d.q(i) = 0.1 * i;
    d.q_ref(i) = d.q(i) + 0.05;
    d.qdot(i) = 0.2;
  }

  const auto full = static_cast<std::size_t>(kNv);
  Eigen::VectorXd tau = Eigen::VectorXd::Constant(kNv, 1234.5);  // stale contents

  // q̇ two channels short: only the channels backed by EVERY input are written.
  ComputePostureTorque(
      d.kp, d.kd,
      PostureInputs{{d.q_ref.data(), full}, {d.q.data(), full}, {d.qdot.data(), full - 2}}, full,
      {tau.data(), full});
  for (int i = 0; i < kNv - 2; ++i)
    EXPECT_TRUE(BitsEqual(tau(i), d.kp * (d.q_ref(i) - d.q(i)) - d.kd * d.qdot(i)))
        << "channel " << i << " must be unaffected by the short tail";
  for (int i = kNv - 2; i < kNv; ++i)
    EXPECT_EQ(tau(i), 0.0) << "channel " << i << " kept a stale torque past the bound";

  // The P form does NOT read q̇, so the same short span must not shrink it.
  Eigen::VectorXd vel = Eigen::VectorXd::Constant(kNv, 1234.5);
  ComputePostureVelocity(
      d.kp, PostureInputs{{d.q_ref.data(), full}, {d.q.data(), full}, {d.qdot.data(), 0}}, full,
      {vel.data(), full});
  for (int i = 0; i < kNv; ++i)
    EXPECT_TRUE(BitsEqual(vel(i), d.kp * (d.q_ref(i) - d.q(i))))
        << "channel " << i << ": an unread input shrank the P form's bound";

  // An n LARGER than the output span cannot overrun it either.
  Eigen::VectorXd small = Eigen::VectorXd::Constant(3, 1234.5);
  ComputePostureTorque(d.kp, d.kd, InputsOf(d, full), full, {small.data(), 3});
  for (int i = 0; i < 3; ++i)
    EXPECT_TRUE(BitsEqual(small(i), d.kp * (d.q_ref(i) - d.q(i)) - d.kd * d.qdot(i)))
        << "channel " << i;
}

// ═══════════════════════════════════════════════════════════════════════════
// Oracle 1b — §7.6 MUST-1, literal pre-extraction form
// ═══════════════════════════════════════════════════════════════════════════

namespace {

struct BandwidthDraw {
  AdmittanceParams outer{};
  ImpedanceParams inner{};
  Eigen::MatrixXd lambda_s{Eigen::MatrixXd::Identity(6, 6)};
  double min_ratio{3.0};
};

// EXACTLY the body of CascadedComplianceController::EvaluateBandwidthSeparation
// at 8526a4c9 (cascaded_compliance_controller.cpp:225-273), rename-only:
//   gains.admittance → d.outer   gains.impedance → d.inner
//   dyn_.LambdaS()   → d.lambda_s
//   bandwidth_ratio_ / bandwidth_ratio_low_ → the returned pair.
BandwidthSeparation PreExtractionBandwidthForm(const BandwidthDraw& d) {
  constexpr int kTaskDim = 6;
  const Eigen::MatrixXd& lambda_s = d.lambda_s;
  double worst = std::numeric_limits<double>::infinity();
  for (int i = 0; i < kTaskDim; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    const double kp_a = d.outer.stiffness[ui];
    if (!(kp_a > 0.0))
      continue;
    const double lambda_d =
        std::max(d.outer.inertia[ui], (i < 3) ? d.outer.min_inertia_lin : d.outer.min_inertia_ang);
    const double lambda_i = lambda_s(i, i);
    const double kp_i = (i < 3) ? d.inner.kp_pos[ui] : d.inner.kp_rot[ui - 3];
    if (!(lambda_d > 0.0) || !(lambda_i > 0.0))
      continue;
    if (!(kp_i > 0.0)) {
      worst = 0.0;
      continue;
    }
    const double omega_a = std::sqrt(kp_a / lambda_d);
    const double omega_i = std::sqrt(kp_i / lambda_i);
    if (!(omega_a > 0.0))
      continue;
    worst = std::min(worst, omega_i / omega_a);
  }
  return BandwidthSeparation{worst, std::isfinite(worst) && (worst < d.min_ratio)};
}

BandwidthSeparation RunBandwidthCore(const BandwidthDraw& d) {
  return EvaluateBandwidthSeparation(d.outer, d.inner, d.lambda_s, d.min_ratio);
}

struct BandwidthCounts {
  int separated{0};      ///< finite ratio at or above the minimum
  int coupled{0};        ///< finite ratio below it (flag set)
  int not_evaluable{0};  ///< ∞ — no axis carried a verdict
  int zero_inner{0};     ///< an ω_i = 0 axis forced the worst to 0
  int skipped_axis{0};   ///< at least one axis dropped out (K_p^a = 0 or Λ ≤ 0)
};

// Draws sweep the configurations §7.6 MUST-1 is written against, including the
// three the early-outs exist for. Λ_S is diagonal-positive most of the time (it
// is an inertia) but is occasionally given a non-positive diagonal entry, which
// is what a degenerate seeding pose produces.
BandwidthDraw RandomBandwidthDraw(std::mt19937& rng, int trial) {
  std::uniform_real_distribution<double> outer_kp(0.0, 400.0);
  std::uniform_real_distribution<double> inner_kp(0.0, 600.0);
  std::uniform_real_distribution<double> inertia(0.01, 8.0);
  std::uniform_real_distribution<double> lam(0.05, 20.0);

  BandwidthDraw d;
  d.lambda_s = Eigen::MatrixXd::Zero(6, 6);
  for (std::size_t i = 0; i < 6; ++i) {
    d.outer.stiffness[i] = outer_kp(rng);
    d.outer.inertia[i] = inertia(rng);
    d.lambda_s(static_cast<int>(i), static_cast<int>(i)) = lam(rng);
  }
  for (std::size_t i = 0; i < 3; ++i) {
    d.inner.kp_pos[i] = inner_kp(rng);
    d.inner.kp_rot[i] = inner_kp(rng);
  }
  d.min_ratio = 3.0;

  switch (trial % 7) {
    case 1:
      d.outer.stiffness = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};  // hand-guiding: no verdict at all
      break;
    case 2:
      d.inner.kp_pos = {{0.0, 0.0, 0.0}};  // ω_i = 0: the WORST separation, not "unevaluable"
      break;
    case 3:
      d.lambda_s(2, 2) = -1.0;  // degenerate seeding pose on one axis
      break;
    case 4:
      d.outer.stiffness[4] = 0.0;  // one axis hand-guided, the rest not
      break;
    case 5:
      // Λ_d driven below the §7.4 floor, so the floor — not the configured value
      // — must be what the ratio uses.
      d.outer.inertia = {{1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9}};
      break;
    case 6:
      // A well-separated cascade — a stiff inner loop under a soft, heavy outer
      // one. A uniform draw essentially never produces this: the verdict is a
      // MIN over six axes, so one mediocre axis is enough to pull it under the
      // threshold, and without this case the "separated" branch of the flag
      // would never run and the comparison would be pinning only the coupled
      // half of the law.
      for (std::size_t i = 0; i < 6; ++i) {
        d.outer.stiffness[i] = 5.0 + static_cast<double>(i);
        d.outer.inertia[i] = 4.0;
        d.lambda_s(static_cast<int>(i), static_cast<int>(i)) = 0.5;
      }
      for (std::size_t i = 0; i < 3; ++i) {
        d.inner.kp_pos[i] = 500.0;
        d.inner.kp_rot[i] = 500.0;
      }
      break;
    default:
      break;
  }
  return d;
}

void TallyBandwidth(const BandwidthDraw& d, const BandwidthSeparation& r, BandwidthCounts* c) {
  if (!std::isfinite(r.ratio))
    ++c->not_evaluable;
  else if (r.low)
    ++c->coupled;
  else
    ++c->separated;
  if (r.ratio == 0.0)
    ++c->zero_inner;
  for (std::size_t i = 0; i < 6; ++i) {
    const int ii = static_cast<int>(i);
    if (!(d.outer.stiffness[i] > 0.0) || !(d.lambda_s(ii, ii) > 0.0)) {
      ++c->skipped_axis;
      break;
    }
  }
}

}  // namespace

TEST(BandwidthSeparation, MatchesThePreExtractionInlineFormBitwise) {
  auto rng = MakeRng();
  BandwidthCounts counts;

  for (int trial = 0; trial < 600; ++trial) {
    const BandwidthDraw d = RandomBandwidthDraw(rng, trial);

    const BandwidthSeparation got = RunBandwidthCore(d);
    const BandwidthSeparation want = PreExtractionBandwidthForm(d);
    TallyBandwidth(d, want, &counts);

    EXPECT_TRUE(BitsEqual(got.ratio, want.ratio))
        << "trial " << trial << ": ratio " << got.ratio << " vs " << want.ratio;
    EXPECT_EQ(got.low, want.low) << "trial " << trial << ": flag";
  }

  // Each early-out measured to have run, or the comparison is pinning one path
  // and calling it the verdict.
  EXPECT_GT(counts.separated, 0) << "no draw was separated — the ordinary path never ran";
  EXPECT_GT(counts.coupled, 0) << "no draw tripped the flag — the low branch never ran";
  EXPECT_GT(counts.not_evaluable, 0) << "no draw was hand-guided on every axis (ratio = ∞)";
  EXPECT_GT(counts.zero_inner, 0) << "the K_p^i = 0 axis (ω_i = 0 ⇒ worst = 0) never ran";
  EXPECT_GT(counts.skipped_axis, 0) << "no draw dropped an axis out of the min()";
}

// A verdict is a MIN over axes, and the axis that carries it must be the worst
// one. A per-axis check the random sweep cannot make: it compares two whole
// verdicts, so a law that reported the BEST axis would agree with an oracle that
// did the same. Here the answer is known in closed form.
TEST(BandwidthSeparation, ReportsTheWorstAxisAndNotAnAverage) {
  BandwidthDraw d;
  d.lambda_s = Eigen::MatrixXd::Identity(6, 6);
  d.outer.min_inertia_lin = 1.0;
  d.outer.min_inertia_ang = 1.0;
  for (std::size_t i = 0; i < 6; ++i) {
    d.outer.stiffness[i] = 1.0;  // ω_a = 1 on every axis
    d.outer.inertia[i] = 1.0;
  }
  // ω_i = √kp_i with Λ_S = I, so the ratio IS √kp_i: axis 1 is the worst at 2.
  d.inner.kp_pos = {{100.0, 4.0, 100.0}};
  d.inner.kp_rot = {{100.0, 100.0, 100.0}};
  d.min_ratio = 3.0;

  const BandwidthSeparation r = RunBandwidthCore(d);
  EXPECT_DOUBLE_EQ(r.ratio, 2.0) << "five separated axes and one coupled one must report the "
                                    "coupled one";
  EXPECT_TRUE(r.low) << "2.0 < 3.0 — the flag must follow the worst axis";

  // Non-vacuity: lift the one bad axis and the same configuration is separated.
  d.inner.kp_pos[1] = 100.0;
  const BandwidthSeparation lifted = RunBandwidthCore(d);
  EXPECT_DOUBLE_EQ(lifted.ratio, 10.0);
  EXPECT_FALSE(lifted.low);
}

// The S4 lesson in its second form: Eigen::Ref arguments bring their own
// dimensions, and eigen_assert cannot be the sensor because the library TU ships
// -DNDEBUG. Axes the caller's Λ_S does not cover must drop out of the min()
// rather than be read past the buffer.
TEST(BandwidthSeparation, UndersizedLambdaSDropsAxesInsteadOfReadingPastIt) {
  BandwidthDraw d;
  d.outer.min_inertia_lin = 1.0;
  d.outer.min_inertia_ang = 1.0;
  for (std::size_t i = 0; i < 6; ++i) {
    d.outer.stiffness[i] = 1.0;
    d.outer.inertia[i] = 1.0;
  }
  d.inner.kp_pos = {{100.0, 100.0, 100.0}};
  d.inner.kp_rot = {{4.0, 4.0, 4.0}};  // the coupled axes live in the ROTATION block
  d.min_ratio = 3.0;

  d.lambda_s = Eigen::MatrixXd::Identity(6, 6);
  const BandwidthSeparation full = RunBandwidthCore(d);
  EXPECT_DOUBLE_EQ(full.ratio, 2.0) << "the rotation axes must reach the min when Λ_S covers them";
  EXPECT_TRUE(full.low);

  // A 3×3 Λ_S: only the translation axes are evaluable, so the verdict is theirs.
  d.lambda_s = Eigen::MatrixXd::Identity(3, 3);
  const BandwidthSeparation partial = RunBandwidthCore(d);
  EXPECT_DOUBLE_EQ(partial.ratio, 10.0) << "an undersized Λ_S must drop the axes it cannot cover";
  EXPECT_FALSE(partial.low);
  EXPECT_TRUE(std::isfinite(partial.ratio))
      << "the covered axes still carry a verdict — dropping is per-axis, not wholesale";

  // Zero-sized Λ_S: nothing is evaluable at all.
  d.lambda_s = Eigen::MatrixXd::Zero(0, 0);
  const BandwidthSeparation none = RunBandwidthCore(d);
  EXPECT_FALSE(std::isfinite(none.ratio));
  EXPECT_FALSE(none.low) << "'not evaluable' must never present as 'coupled'";
}

// ═══════════════════════════════════════════════════════════════════════════
// RT contract for both cores
// ═══════════════════════════════════════════════════════════════════════════

// RT-1. It takes BOTH sensors to be one gate:
//   • the operator-new counter — sees a std::vector or a header-inline helper
//     (a bare `new`+immediate-`delete` pair would be elided, so a mutation has
//     to be a real runtime-sized allocation reaching an external sink);
//   • the Eigen tripwire — the only one that can see a fixed-size argument or
//     temporary regressing to a runtime-sized Eigen type, invisible to
//     operator-new interposition because Eigen's internal::aligned_malloc calls
//     std::malloc directly. The verdict takes Λ_S as a Ref, which is exactly the
//     shape that heap-copies if its stride assumption is broken — binding
//     `lambda_s.diagonal()` to a plain Ref<const VectorXd> would allocate here.
// Both are armed by RAII, so an early return cannot leave a live gate behind.
TEST(PostureCore, IsAllocationFree) {
  auto rng = MakeRng();

  std::vector<PostureDraw> draws;
  std::vector<BandwidthDraw> bw_draws;
  draws.reserve(64);
  bw_draws.reserve(64);
  for (int trial = 1; trial <= 64; ++trial) {
    draws.push_back(RandomDraw(rng, trial));
    bw_draws.push_back(RandomBandwidthDraw(rng, trial));
  }

  // Output buffers and the gains POD live outside the gate, as the adapter's do
  // (they are members, sized at configure time).
  const auto n = static_cast<std::size_t>(kNv);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(kNv);
  Eigen::VectorXd vel = Eigen::VectorXd::Zero(kNv);
  CascadedComplianceController::Gains g;
  double sink = 0.0;
  ComputePostureTorque(1.0, 1.0, InputsOf(draws.front(), n), n, {tau.data(), n});  // warm up

  std::size_t new_calls = 0;
  std::uint64_t eigen_allocs = 0;
  {
    rtc::testing::ScopedAllocGate heap_gate;
    rtc::testing::ScopedNoMalloc eigen_gate;
    for (std::size_t k = 0; k < draws.size(); ++k) {
      const PostureDraw& d = draws[k];
      // Marshalled from the gains POD the adapter actually holds, so a
      // regression that made nullspace_kp something other than a double would
      // show up here as an allocation on the way in.
      g.nullspace_kp = d.kp;
      g.nullspace_kd = d.kd;
      ComputePostureTorque(g.nullspace_kp, g.nullspace_kd, InputsOf(d, n), n, {tau.data(), n});
      ComputePostureVelocity(g.nullspace_kp, InputsOf(d, n), n, {vel.data(), n});
      sink += tau(0) + vel(0);

      const BandwidthDraw& b = bw_draws[k];
      g.admittance = b.outer;
      g.impedance = b.inner;
      const BandwidthSeparation r =
          EvaluateBandwidthSeparation(g.admittance, g.impedance, b.lambda_s, g.min_bandwidth_ratio);
      sink += std::isfinite(r.ratio) ? r.ratio : 0.0;
    }
    new_calls = heap_gate.count();
    eigen_allocs = eigen_gate.violations();
  }

  EXPECT_EQ(new_calls, 0u) << "an S6 core reached operator new on the RT path (RT-1)";
  EXPECT_EQ(eigen_allocs, 0u) << "an S6 core made Eigen allocate on the RT path (RT-1) — the most "
                                 "likely cause is the Λ_S argument acquiring a stride the Ref "
                                 "cannot map";
  EXPECT_TRUE(std::isfinite(sink)) << "sink kept the calls from being optimised away";
}

// ═══════════════════════════════════════════════════════════════════════════
// Oracle 2 — the shim (prototype of the S7 integration-layer binding)
// ═══════════════════════════════════════════════════════════════════════════

namespace {

// What one tick of the adapter puts on device 0, plus the diagnostics that are
// the ONLY window onto the two gates. Pinning those separately is what localises
// a failure to a gate rather than leaving the whole tick suspect.
struct ShimOutput {
  Eigen::VectorXd tau_dev{};
  Vec6 pose_error{Vec6::Zero()};
  Vec6 nu_c{Vec6::Zero()};
  bool nullspace_active{false};
  double bandwidth_ratio{std::numeric_limits<double>::infinity()};
  bool bandwidth_ratio_low{false};
};

// Owns the model, both loops and the torque assembly (STRUCTURE) and calls the
// two cores (ALGORITHM). Mirrors CascadedComplianceController::Compute() step
// for step from the joint-state copy down to the device-order torque; any
// divergence here is a divergence the future binding would ship.
//
// This is the composition the plan predicted (§S6 R5): S5's AdmittanceShim
// instantiates the outer AdmittanceIntegrator rather than re-deriving it, and
// S4's ImpedanceShim replicates the Λ_S / Nᵀ / τ tail. Nothing new was designed.
class CascadeShim {
 public:
  CascadeShim() {
    handle_ = MakeHandle(model_);
    tip_ = static_cast<pinocchio::FrameIndex>(model_->nframes - 1);
    nv_ = handle_->nv();

    J_full_ = Eigen::MatrixXd::Zero(6, nv_);
    M_ = Eigen::MatrixXd::Zero(nv_, nv_);
    llt_M_ = Eigen::LLT<Eigen::MatrixXd>(nv_);
    qdot_ = Eigen::VectorXd::Zero(nv_);
    q_dev_ = Eigen::VectorXd::Zero(nv_);
    qdot_dev_ = Eigen::VectorXd::Zero(nv_);
    q_null_ = Eigen::VectorXd::Zero(nv_);
    gravity_ = Eigen::VectorXd::Zero(nv_);
    tau_ = Eigen::VectorXd::Zero(nv_);
    tau_dev_ = Eigen::VectorXd::Zero(nv_);
    tau_posture_ = Eigen::VectorXd::Zero(nv_);
    tau_posture_dev_ = Eigen::VectorXd::Zero(nv_);
    tau_null_ = Eigen::VectorXd::Zero(nv_);
    dyn_.Resize(nv_, CascadedComplianceController::kTaskDim);
  }

  [[nodiscard]] int nv() const { return nv_; }

  /// Mirror of set_gains()'s side effect: the adapter re-arms the §7.6
  /// evaluation whenever a new POD reaches the SeqLock.
  void RequestBandwidthEval() { bw_pending_ = true; }

  // One tick. `f_lwa`, `wrench_live` are the conditioned wrench and the pipeline
  // status the adapter produced on THIS tick (diag.wrench_lwa, diag.wrench_valid
  // && diag.bias_calibrated) — the one lane the shim borrows rather than
  // replicates, as S4's and S5's shims did. Returns the torque in DEVICE order
  // BEFORE the §10.5 safety layer, which the caller asserts inert.
  ShimOutput Step(const CascadedComplianceController::Gains& g, const rtc::ControllerState& state,
                  const std::array<double, 6>& f_lwa, bool wrench_live) {
    constexpr int kTaskDim = CascadedComplianceController::kTaskDim;
    const auto& dev0 = state.devices[0];
    const double dt = state.dt;

    std::array<double, kMaxDeviceChannels> q_buf{};
    for (int i = 0; i < nv_; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      q_buf[ui] = dev0.positions[ui];
      q_dev_(i) = dev0.positions[ui];
      qdot_dev_(i) = dev0.velocities[ui];
    }
    std::span<const double> q_span(q_buf.data(), static_cast<std::size_t>(nv_));

    handle_->ComputeJacobians(q_span);
    handle_->GetFrameJacobian(tip_, pinocchio::LOCAL_WORLD_ALIGNED, J_full_);
    handle_->ReorderInput(std::span<const double>(qdot_dev_.data(), static_cast<std::size_t>(nv_)),
                          qdot_);
    tcp_vel_.noalias() = J_full_ * qdot_;
    const pinocchio::SE3& tcp = handle_->GetFramePlacement(tip_);

    // Seed X_d and the posture target from the measurement (§10.7). The
    // integrator Reset is part of that seed: an activation must not inherit a
    // deviation accrued while the controller was not running.
    bool just_seeded = false;
    if (!initialized_) {
      goal_pose_ = tcp;
      for (int i = 0; i < nv_; ++i)
        q_null_(i) = q_dev_(i);
      integrator_.Reset();
      activation_elapsed_ = 0.0;
      wrench_elapsed_ = 0.0;
      just_seeded = true;
      initialized_ = true;
    }

    const double ramp = g.activation_ramp_time;
    alpha_ = (ramp <= 0.0) ? 1.0 : std::min(1.0, activation_elapsed_ / ramp);
    const double alpha_wrench = (ramp <= 0.0) ? 1.0 : std::min(1.0, wrench_elapsed_ / ramp);
    if (alpha_ > 0.0 && alpha_ < 1.0)
      ++ramped_ticks_;

    Vec6 f_raw;
    for (int i = 0; i < 6; ++i)
      f_raw(i) = f_lwa[static_cast<std::size_t>(i)];
    if (f_raw.norm() > 0.0)
      ++wrench_ticks_;
    const Vec6 f_ext = alpha_wrench * f_raw;

    // ── OUTER: §7.2 compliant frame ────────────────────────────────────────
    const rtc::compliance::AdmittanceStatus ast = integrator_.Step(g.admittance, f_ext, dt);
    if (!ast.finite)
      ++nonfinite_steps_;
    compliant_pose_.rotation() = integrator_.rotation() * goal_pose_.rotation();
    compliant_pose_.translation() = goal_pose_.translation() + integrator_.translation();
    const Vec6& nu_c = integrator_.velocity();

    // ── INNER: §6.2 impedance tracking of the compliant frame ──────────────
    const Vec6 e = rtc::math::se3::computePoseError(tcp, compliant_pose_,
                                                    rtc::math::se3::ErrorType::SplitWorld);
    const Vec6 f_task =
        rtc::compliance::ComputeImpedanceForce(g.impedance, e, tcp_vel_, nu_c, alpha_, kTaskDim);

    handle_->ComputeGeneralizedGravity(q_span);
    gravity_ = handle_->GetGeneralizedGravity();

    // ── Λ_S / Nᵀ, the §7.6 verdict and the posture task ────────────────────
    bool dyn_ok = true;
    // #277's `FloorPostureGain` sits immediately above this line in the adapter
    // and is deliberately not mirrored: it is a binding-side clamp, and every
    // gain this shim is driven with is non-negative, where the floor is the
    // identity. A negative gain here would make the two sides disagree — floor
    // at the CALL SITE if one is ever added.
    const bool nullspace_active =
        (nv_ > kTaskDim) && (g.nullspace_kp != 0.0 || g.nullspace_kd != 0.0);
    const bool bw_pending = bw_pending_;
    bw_pending_ = false;
    const bool bw_evaluate = just_seeded || bw_pending;
    if (bw_evaluate) {
      bandwidth_ratio_ = std::numeric_limits<double>::infinity();
      bandwidth_ratio_low_ = false;
      ++bw_attempts_;
    }
    if (nullspace_active || bw_evaluate) {
      handle_->ComputeMassMatrix(q_span);
      M_ = handle_->GetMassMatrix();
      M_.triangularView<Eigen::StrictlyLower>() =
          M_.triangularView<Eigen::StrictlyUpper>().transpose();
      llt_M_.compute(M_);
      if (llt_M_.info() != Eigen::Success) {
        if (nullspace_active)
          dyn_ok = false;
        if (bw_evaluate) {
          bw_pending_ = true;
          ++bw_retries_;
        }
      } else {
        const rtc::compliance::TaskDynamics::Result r = dyn_.Compute(
            J_full_, llt_M_, g.singularity_threshold, std::max(kMinMaxDamping, g.max_damping));
        if (bw_evaluate && r.ok) {
          const BandwidthSeparation bw = EvaluateBandwidthSeparation(
              g.admittance, g.impedance, dyn_.LambdaS(), g.min_bandwidth_ratio);
          bandwidth_ratio_ = bw.ratio;
          bandwidth_ratio_low_ = bw.low;
          ++bw_evaluations_;
        } else if (bw_evaluate) {
          bw_pending_ = true;
          ++bw_retries_;
        }
        if (nullspace_active) {
          dyn_ok = r.ok;
          if (dyn_ok) {
            const auto nvu = static_cast<std::size_t>(nv_);
            ComputePostureTorque(
                g.nullspace_kp, g.nullspace_kd,
                PostureInputs{{q_null_.data(), nvu}, {q_dev_.data(), nvu}, {qdot_dev_.data(), nvu}},
                nvu, {tau_posture_dev_.data(), nvu});
            handle_->ReorderInput(std::span<const double>(tau_posture_dev_.data(), nvu),
                                  tau_posture_);
            dyn_.ProjectNullspace(tau_posture_, tau_null_);
            ++nullspace_ticks_;
            peak_posture_ = std::max(peak_posture_, tau_posture_dev_.cwiseAbs().maxCoeff());
          }
        }
      }
    }

    // ── τ = Jᵀ f_task + α Nᵀ τ_posture + ĝ ─────────────────────────────────
    tau_.noalias() = J_full_.transpose() * f_task;
    if (dyn_ok && nullspace_active)
      tau_.noalias() += alpha_ * tau_null_;
    if (dyn_ok) {
      tau_ += gravity_;
    } else {
      tau_ = gravity_;
    }
    handle_->ReorderOutput(tau_, std::span<double>(tau_dev_.data(), static_cast<std::size_t>(nv_)));

    // The adapter advances the ramps only on a clean control tick, and α_wrench
    // additionally waits for the pipeline to have something to ramp in.
    activation_elapsed_ += dt;
    if (wrench_live)
      wrench_elapsed_ += dt;

    ShimOutput out;
    out.tau_dev = tau_dev_;
    out.pose_error = e;
    out.nu_c = nu_c;
    out.nullspace_active = nullspace_active && dyn_ok;
    out.bandwidth_ratio = bandwidth_ratio_;
    out.bandwidth_ratio_low = bandwidth_ratio_low_;
    return out;
  }

  // Branch counters — a cross-check that never took the branch it was written
  // for is vacuous with respect to the gate it exists to pin (the S2a lesson).
  [[nodiscard]] int nullspace_ticks() const { return nullspace_ticks_; }

  [[nodiscard]] double peak_posture() const { return peak_posture_; }

  [[nodiscard]] int wrench_ticks() const { return wrench_ticks_; }

  [[nodiscard]] int ramped_ticks() const { return ramped_ticks_; }

  [[nodiscard]] int nonfinite_steps() const { return nonfinite_steps_; }

  [[nodiscard]] int bw_attempts() const { return bw_attempts_; }

  [[nodiscard]] int bw_evaluations() const { return bw_evaluations_; }

  [[nodiscard]] int bw_retries() const { return bw_retries_; }

  [[nodiscard]] double alpha() const { return alpha_; }

 private:
  std::shared_ptr<const pinocchio::Model> model_;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle_;
  pinocchio::FrameIndex tip_{0};
  int nv_{0};

  rtc::compliance::AdmittanceIntegrator integrator_;
  rtc::compliance::TaskDynamics dyn_;

  bool initialized_{false};
  bool bw_pending_{false};
  double activation_elapsed_{0.0};
  double wrench_elapsed_{0.0};
  double alpha_{0.0};
  double bandwidth_ratio_{std::numeric_limits<double>::infinity()};
  bool bandwidth_ratio_low_{false};
  int nullspace_ticks_{0};
  int wrench_ticks_{0};
  int ramped_ticks_{0};
  int nonfinite_steps_{0};
  int bw_attempts_{0};
  int bw_evaluations_{0};
  int bw_retries_{0};
  double peak_posture_{0.0};

  pinocchio::SE3 goal_pose_{pinocchio::SE3::Identity()};
  pinocchio::SE3 compliant_pose_{pinocchio::SE3::Identity()};
  Eigen::MatrixXd J_full_;
  Eigen::MatrixXd M_;
  Eigen::LLT<Eigen::MatrixXd> llt_M_;
  Vec6 tcp_vel_{Vec6::Zero()};
  Eigen::VectorXd qdot_;
  Eigen::VectorXd q_dev_;
  Eigen::VectorXd qdot_dev_;
  Eigen::VectorXd q_null_;
  Eigen::VectorXd gravity_;
  Eigen::VectorXd tau_;
  Eigen::VectorXd tau_dev_;
  Eigen::VectorXd tau_posture_;
  Eigen::VectorXd tau_posture_dev_;
  Eigen::VectorXd tau_null_;
};

// A transparent wrench source: no deadband, no filter, no bias calibration. This
// slice is about the posture law and the §7.6 verdict, so every upstream stage
// that could re-shape f_ext is switched off rather than replicated (each has its
// own test in test_cascaded_compliance_controller.cpp).
std::string CrossCheckYaml() {
  return R"(
external_wrench:
  enabled: true
  filter_enabled: false
  bias_calibration_samples: 0
  deadband: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  max: [1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6]
)";
}

CascadedComplianceController::Gains CrossCheckGains() {
  CascadedComplianceController::Gains g;
  // PAIRWISE-DISTINCT gains, not isotropic blocks. Oracle 2 exists to catch
  // WIRING errors, and a permutation inside one block is bit-for-bit invisible
  // when all three values are equal.
  g.impedance.kp_pos = {{140.0, 120.0, 160.0}};
  g.impedance.kd_pos = {{20.0, 17.0, 23.0}};
  g.impedance.kp_rot = {{14.0, 11.0, 17.0}};
  g.impedance.kd_rot = {{5.0, 3.5, 6.5}};
  g.admittance.stiffness = {{220.0, 180.0, 260.0, 6.0, 4.5, 7.5}};
  g.admittance.inertia = {{2.5, 2.0, 3.0, 0.06, 0.05, 0.07}};
  g.nullspace_kp = 2.0;
  g.nullspace_kd = 0.5;
  g.activation_ramp_time = 0.0;  // α = 1 unless a case says otherwise
  // The §10.5 rate limit is a SAFETY-LAYER parameter, not a law gain, and a
  // bitwise comparison is void the moment it fires — the shim stops at the law.
  // Raising it moves the OPERATING POINT (the S3a/S4 lesson) instead of shrinking
  // the wrench that drives the compliant frame. That it stays inert is still
  // asserted every tick, not assumed.
  g.max_torque_rate = 1.0e6;
  return g;
}

struct CrossCheckStats {
  double peak_tau{0.0};
  double peak_err{0.0};
  double peak_nu_c{0.0};
  int nullspace_active_ticks{0};
};

// Which measured state the pair is driven through. kSweep is the shared fixture
// sweep every other suite uses; kSingularHold parks the arm at the all-zero
// configuration, where serial_7dof is fully extended and J drops rank — the
// pose the §6.5 damping exists for, and the only input that makes the Λ_S
// factorisation genuinely fail rather than merely become ill-conditioned.
enum class Drive { kSweep, kSingularHold };

// Drive adapter and shim through the same tick sequence and compare the torque,
// both core inputs and BOTH gate windows, bit for bit.
void RunCrossCheck(CascadedComplianceController& ctrl, CascadeShim& shim,
                   const CascadedComplianceController::Gains& g, int ticks, CrossCheckStats* stats,
                   Drive drive = Drive::kSweep) {
  const int nv = shim.nv();
  auto state = MakeState(nv, kDt);

  for (int tick = 0; tick < ticks; ++tick) {
    if (drive == Drive::kSweep) {
      FillSweep(state, nv, tick, kDt);
    } else {
      for (int j = 0; j < nv; ++j) {
        const auto uj = static_cast<std::size_t>(j);
        state.devices[0].positions[uj] = 0.0;
        state.devices[0].velocities[uj] = 0.0;
      }
    }

    // A wrench that changes sign and magnitude, so the compliant frame keeps
    // moving and ν_c is not a constant the comparison could absorb.
    const double phase = 0.05 * tick;
    const std::array<double, 6> w{
        5.0 * std::cos(phase), 3.5 * std::sin(phase), -2.5, 0.4 * std::sin(phase), -0.3, 0.2};
    ctrl.SetExternalWrench(std::span<const double, 6>(w.data(), 6));

    const auto out = ctrl.Compute(state);
    const auto diag = ctrl.GetDiagnosticsForTesting();
    ASSERT_TRUE(diag.control_valid) << "tick " << tick << ": the adapter did not run the law";
    // A fired safety layer would silently void the comparison rather than fail
    // it informatively — assert inert, do not assume (S1 harness contract 5).
    ASSERT_FALSE(diag.saturated) << "tick " << tick << ": §10.5 saturation fired";
    ASSERT_FALSE(diag.rate_limited) << "tick " << tick << ": §10.5 rate limit fired";

    const ShimOutput shim_out =
        shim.Step(g, state, diag.wrench_lwa, diag.wrench_valid && diag.bias_calibrated);

    for (int j = 0; j < nv; ++j) {
      const auto uj = static_cast<std::size_t>(j);
      const double adapter = out.devices[0].commands[uj];
      ASSERT_LT(std::abs(adapter), rtc::kDefaultMaxJointTorque)
          << "output clamp fired at tick " << tick << " ch " << j << " — comparison invalid";
      EXPECT_TRUE(BitsEqual(adapter, shim_out.tau_dev(j)))
          << "tick " << tick << " channel " << j << ": adapter " << adapter << " vs shim "
          << shim_out.tau_dev(j);
      stats->peak_tau = std::max(stats->peak_tau, std::abs(adapter));
    }

    for (int i = 0; i < 6; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      EXPECT_TRUE(BitsEqual(diag.pose_error[ui], shim_out.pose_error(i)))
          << "tick " << tick << " pose-error axis " << i << " (the inner law's e input)";
      EXPECT_TRUE(BitsEqual(diag.compliant_velocity[ui], shim_out.nu_c(i)))
          << "tick " << tick << " compliant-velocity axis " << i;
      stats->peak_err = std::max(stats->peak_err, std::abs(diag.pose_error[ui]));
      stats->peak_nu_c = std::max(stats->peak_nu_c, std::abs(diag.compliant_velocity[ui]));
    }

    // The posture gate. With the gains zero the ungated path would contribute
    // signed zeros the torque sum absorbs exactly, so this flag is the ONLY
    // observable separating "the branch was skipped" from "the branch ran and
    // added nothing" (S5 M4 measured exactly that false green).
    EXPECT_EQ(diag.nullspace_active, shim_out.nullspace_active)
        << "tick " << tick << ": adapter and shim disagree on the posture gate";
    if (diag.nullspace_active)
      ++stats->nullspace_active_ticks;

    // The §7.6 verdict. Bitwise, and on EVERY tick rather than only the ones
    // that evaluate — the gate's failure mode is a stale value that persists, so
    // the ticks in between are where a mis-wired gate shows.
    EXPECT_TRUE(BitsEqual(diag.bandwidth_ratio, shim_out.bandwidth_ratio))
        << "tick " << tick << ": ratio " << diag.bandwidth_ratio << " vs "
        << shim_out.bandwidth_ratio;
    EXPECT_EQ(diag.bandwidth_ratio_low, shim_out.bandwidth_ratio_low)
        << "tick " << tick << ": §7.6 flag";
  }
}

// Shim-only driver for the §7.6 evaluation-gate cases (#236 S7c-2, 분류 B).
//
// The one lane RunCrossCheck borrows from the adapter is the CONDITIONED wrench
// (diag.wrench_lwa), and the §7.6 verdict does not read it:
// EvaluateBandwidthSeparation is a function of the two gain sets and Λ_S, i.e.
// of q alone. So these cases can drive the same signal straight in as an
// already-LOCAL_WORLD_ALIGNED wrench and drop their last adapter dependency,
// without weakening what they assert. (The CrossCheckYaml pipeline the adapter
// runs is a pass-through by construction — filter off, no bias, zero deadband,
// 1e6 saturation — so the difference is the frame transform, which changes where
// the compliant frame goes and nothing about the verdict.)
//
// Returns the last tick's ShimOutput: the §7.6 fields are what the cases assert,
// and reading them from the OUTPUT rather than from a counter is what keeps
// "the published figure follows the gains" a claim about the published figure.
ShimOutput RunShimOnly(CascadeShim& shim, const CascadedComplianceController::Gains& g, int ticks,
                       Drive drive = Drive::kSweep) {
  const int nv = shim.nv();
  auto state = MakeState(nv, kDt);
  ShimOutput last{};

  for (int tick = 0; tick < ticks; ++tick) {
    if (drive == Drive::kSweep) {
      FillSweep(state, nv, tick, kDt);
    } else {
      for (int j = 0; j < nv; ++j) {
        const auto uj = static_cast<std::size_t>(j);
        state.devices[0].positions[uj] = 0.0;
        state.devices[0].velocities[uj] = 0.0;
      }
    }

    const double phase = 0.05 * tick;
    const std::array<double, 6> w{
        5.0 * std::cos(phase), 3.5 * std::sin(phase), -2.5, 0.4 * std::sin(phase), -0.3, 0.2};
    last = shim.Step(g, state, w, /*wrench_live=*/true);
  }
  return last;
}

// One posture-gate combination end to end. The gate is `kp != 0 || kd != 0`, so
// three of the four corners are meaningfully different: both gains, stiffness
// only, damping only (which is where the PD form produces −0.0·Δq on the seed
// tick), and neither — the closed gate, which contributes nothing numerically
// and is therefore pinned by diag.nullspace_active alone.
void RunPostureCombination(double nullspace_kp, double nullspace_kd, bool expect_gate_open) {
  auto g = CrossCheckGains();
  g.nullspace_kp = nullspace_kp;
  g.nullspace_kd = nullspace_kd;

  CascadedComplianceController ctrl(Serial7dof(), g);
  ctrl.SetControlRate(1.0 / kDt);
  ctrl.LoadConfig(YAML::Load(CrossCheckYaml()));
  ctrl.OnDeviceConfigsSet();
  ctrl.set_gains(g);  // LoadConfig owns the wiring; the POD under test comes from here
  ASSERT_TRUE(ctrl.external_wrench_enabled());

  CascadeShim shim;
  ASSERT_EQ(shim.nv(), kNv) << "fixture must be redundant (nv > 6) for a non-trivial null space";
  shim.RequestBandwidthEval();  // mirrors the set_gains() above

  CrossCheckStats stats;
  RunCrossCheck(ctrl, shim, g, /*ticks=*/60, &stats);

  EXPECT_GT(stats.peak_tau, 1e-3) << "every compared torque was ~0 — the cross-check is vacuous";
  EXPECT_GT(stats.peak_err, 1e-6) << "the pose error never left zero — the inner law was inert";
  EXPECT_GT(stats.peak_nu_c, 1e-6) << "the compliant frame never moved — the outer loop was inert";
  EXPECT_GT(shim.wrench_ticks(), 0) << "f_ext was zero on every tick — the outer loop was driven "
                                       "by nothing";
  EXPECT_EQ(shim.nonfinite_steps(), 0) << "the §7.2 integrator refused a step — not the clean-tick "
                                          "regime this suite compares";
  EXPECT_GT(shim.bw_evaluations(), 0) << "the §7.6 verdict was never computed — the ratio lane "
                                         "pins only its initial value";

  if (expect_gate_open) {
    EXPECT_GT(shim.nullspace_ticks(), 0) << "the posture branch never fired";
    EXPECT_GT(stats.nullspace_active_ticks, 0) << "the adapter never reported the gate open";
    EXPECT_GT(shim.peak_posture(), 1e-9)
        << "the posture branch fired but contributed ~0 — it pins nothing";
  } else {
    EXPECT_EQ(shim.nullspace_ticks(), 0) << "the posture task ran with both gains zero";
    EXPECT_EQ(stats.nullspace_active_ticks, 0)
        << "the adapter reported the gate open with both gains zero";
  }
}

}  // namespace

TEST(PostureCore, CoreDrivenShimMatchesTheAdapterBitwiseWithBothGains) {
  RunPostureCombination(/*nullspace_kp=*/2.0, /*nullspace_kd=*/0.5, /*expect_gate_open=*/true);
}

TEST(PostureCore, CoreDrivenShimMatchesTheAdapterBitwiseWithStiffnessOnly) {
  RunPostureCombination(/*nullspace_kp=*/2.0, /*nullspace_kd=*/0.0, /*expect_gate_open=*/true);
}

// K_p = 0 with K_d ≠ 0 opens the gate through its SECOND disjunct, and it is the
// configuration in which the P term is `0.0 · Δq` — a signed zero on every
// channel where the arm has drifted negative. The torque comparison would be
// green either way; what this case adds is that the adapter and the shim reach
// that state through the same branch.
TEST(PostureCore, CoreDrivenShimMatchesTheAdapterBitwiseWithDampingOnly) {
  RunPostureCombination(/*nullspace_kp=*/0.0, /*nullspace_kd=*/0.5, /*expect_gate_open=*/true);
}

// The gate CLOSED. Numerically inert — an adapter that lost the gate would
// scatter `0.0·Δq − 0.0·q̇` through Nᵀ and add exact zeros, leaving every torque
// bit unchanged (S5 M4 measured precisely this false green). diag.nullspace_active
// is the whole sensor, and the case exists so that flag has a negative to report.
TEST(PostureCore, CoreDrivenShimMatchesTheAdapterBitwiseWithThePostureGateClosed) {
  RunPostureCombination(/*nullspace_kp=*/0.0, /*nullspace_kd=*/0.0, /*expect_gate_open=*/false);
}

// The activation ramp touches the posture term's OUTPUT (α·Nᵀτ) and the inner
// law's input (α inside §6.2), on a clock the outer loop does NOT share. With
// the ramp retired in every case above, a binding that ramped the posture torque
// on the wrong clock would still match, so this case runs both clocks partway
// and asserts partial-α ticks actually occurred.
TEST(PostureCore, CoreDrivenShimMatchesTheAdapterBitwiseThroughTheActivationRamp) {
  auto g = CrossCheckGains();
  g.activation_ramp_time = 0.05;  // 25 ticks at 2 ms — partway through most of the run

  CascadedComplianceController ctrl(Serial7dof(), g);
  ctrl.SetControlRate(1.0 / kDt);
  ctrl.LoadConfig(YAML::Load("activation_ramp_time: 0.05" + CrossCheckYaml()));
  ctrl.OnDeviceConfigsSet();
  ctrl.set_gains(g);

  CascadeShim shim;
  shim.RequestBandwidthEval();
  CrossCheckStats stats;
  RunCrossCheck(ctrl, shim, g, /*ticks=*/40, &stats);

  EXPECT_GT(stats.peak_tau, 1e-3) << "every compared torque was ~0 — the cross-check is vacuous";
  EXPECT_GT(shim.ramped_ticks(), 0)
      << "α was never strictly between 0 and 1 — the ramp wiring is not being pinned";
  EXPECT_DOUBLE_EQ(shim.alpha(), 1.0) << "the ramp never completed — the run is too short";
  EXPECT_GT(stats.nullspace_active_ticks, 0) << "the posture gate never opened under the ramp";
}

// ═══════════════════════════════════════════════════════════════════════════
// The §7.6 evaluation gate — the two ways it reopens
// ═══════════════════════════════════════════════════════════════════════════

// Re-evaluation on a gain change. Retargeted onto the shim in #236 S7c-2
// (분류 B): test_cascaded_compliance_controller.cpp used to pin the adapter's
// own behaviour here and this case pinned that the SHIM's copy of the gate
// agreed. With the adapter retiring, the shim IS the owner of the stateful
// contract — the gate reopens on a gain change, and the published figure
// follows the gains it was handed rather than the ones it was seeded with.
TEST(BandwidthSeparation, ShimReEvaluatesTheRatioWhenTheGainsChange) {
  auto g = CrossCheckGains();

  CascadeShim shim;
  shim.RequestBandwidthEval();
  const ShimOutput first = RunShimOnly(shim, g, /*ticks=*/20);
  const double initial = first.bandwidth_ratio;
  ASSERT_TRUE(std::isfinite(initial)) << "the first activation produced no verdict to retire";
  ASSERT_GT(shim.bw_evaluations(), 0) << "nothing was ever evaluated — the case is vacuous";

  // Retire the inner loop's bandwidth: same cascade, an inner stiffness far
  // below the outer one. The published figure must FOLLOW the gains.
  g.impedance.kp_pos = {{1.0, 1.0, 1.0}};
  g.impedance.kp_rot = {{0.1, 0.1, 0.1}};
  shim.RequestBandwidthEval();

  const int before_soft = shim.bw_evaluations();
  const ShimOutput soft = RunShimOnly(shim, g, /*ticks=*/20);

  EXPECT_GT(shim.bw_evaluations(), before_soft) << "the gate never reopened for the new gains";
  EXPECT_LT(soft.bandwidth_ratio, initial)
      << "the report still describes the retired gains — a softer inner loop must LOWER the ratio";
  EXPECT_TRUE(soft.bandwidth_ratio_low)
      << "an inner loop two orders below the outer one is coupled (ratio = "
      << soft.bandwidth_ratio << ")";

  // The threshold is a gain too, and the flag has to follow IT rather than a
  // constant: restore the inner stiffness and drop min_bandwidth_ratio below the
  // resulting figure. Without this the suite would only ever have seen the flag
  // set, which a field wired to `true` would also satisfy.
  g.impedance = CrossCheckGains().impedance;
  g.min_bandwidth_ratio = 0.0;
  shim.RequestBandwidthEval();

  const int before_restore = shim.bw_evaluations();
  const ShimOutput restored = RunShimOnly(shim, g, /*ticks=*/20);

  EXPECT_GT(shim.bw_evaluations(), before_restore) << "the gate never reopened for the third set";
  EXPECT_GT(restored.bandwidth_ratio, soft.bandwidth_ratio)
      << "restoring the inner stiffness did not raise the ratio back";
  EXPECT_FALSE(restored.bandwidth_ratio_low)
      << "the flag ignored min_bandwidth_ratio = 0 — it is not reading the threshold from the "
         "gains it was handed";
  EXPECT_EQ(shim.bw_retries(), 0) << "a healthy pose must not need a retry";
}

// The retry path. When the evaluation cannot be FORMED the owner must publish
// "not evaluable" (∞, flag clear) and try again next tick, never leave the
// previous activation's number in place under gains that no longer apply.
//
// The failure is the REAL one, not an injected NaN: at the all-zero
// configuration serial_7dof is fully extended with its four Z axes parallel, J
// drops to rank 5 and Λ_S⁻¹ = J M⁻¹ Jᵀ is singular — so with the §6.5 damping
// retired (σ₀ = 0 ⇒ λ² = 0) the Cholesky fails at every tick.
//
// (A non-finite gain would NOT do: Eigen's LLT rejects a non-positive pivot, and
// a NaN pivot compares false against `<= 0`, so a NaN λ² factorises "successfully"
// and the verdict then drops every axis on its own Λ_S guard. The retry path is
// only reachable through a genuinely indefinite matrix.)
//
// The posture gains are retired so the tick stays otherwise clean: dyn_ok is
// only assigned under the posture gate, so with that gate closed a failed Λ_S
// costs the REPORT and nothing else — the separation §7.6 asks for ("a
// rank-deficient seeding pose must cost the bandwidth REPORT, not the
// activation"). Retargeted onto the shim in #236 S7c-2 (분류 B).
TEST(BandwidthSeparation, ShimRetriesAfterAFailedEvaluation) {
  auto g = CrossCheckGains();
  g.nullspace_kp = 0.0;
  g.nullspace_kd = 0.0;
  g.singularity_threshold = 0.0;  // no DLS shell: A stays exactly J M⁻¹ Jᵀ

  CascadeShim shim;
  shim.RequestBandwidthEval();
  const ShimOutput failed = RunShimOnly(shim, g, /*ticks=*/12, Drive::kSingularHold);

  EXPECT_FALSE(std::isfinite(failed.bandwidth_ratio))
      << "a Λ_S that cannot be factorised must publish 'not evaluable', not a number";
  EXPECT_FALSE(failed.bandwidth_ratio_low) << "'not evaluable' must never present as 'coupled'";
  EXPECT_GT(shim.bw_retries(), 1) << "the evaluation was not retried — one failure and the gate "
                                     "would stay shut for the rest of the activation";
  EXPECT_EQ(shim.bw_evaluations(), 0) << "a verdict was computed despite the failed solve";
  // §7.6 is a DIAGNOSTIC (D20): a failed report must not stop the law. The
  // adapter said this with diag.control_valid; on this side the equivalent
  // observable is that the tick still produced a finite torque.
  EXPECT_TRUE(failed.tau_dev.allFinite())
      << "the failed REPORT poisoned the torque — §7.6 must cost the report and nothing else";

  // Non-vacuity, and the recovery: restore the §6.5 damping shell — at the SAME
  // singular pose — and the very next evaluation must succeed. Without this the
  // assertions above would hold just as well for a shim that never evaluates at
  // all, and holding the pose fixed keeps the contrast about the damping rather
  // than about having moved off the singularity.
  g.singularity_threshold = 0.02;
  shim.RequestBandwidthEval();
  const ShimOutput recovered = RunShimOnly(shim, g, /*ticks=*/5, Drive::kSingularHold);

  EXPECT_GT(shim.bw_evaluations(), 0) << "the retry never succeeded once the pose was evaluable";
  EXPECT_TRUE(std::isfinite(recovered.bandwidth_ratio))
      << "the verdict stayed at 'not evaluable' after the cause was removed";
}
