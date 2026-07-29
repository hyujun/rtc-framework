// ── Convergence harness: demo_task_controller's inline DLS → DifferentialIk ──
// Issue #282, and the LAST constant-λ spelling in the repo. Unlike #236 S2b/S3b
// this one is WIRED: three shipped robot configs (ur5e_p1a, ur5e_p1b,
// iiwa7_leap) run this control law, so "the extraction changed nothing" is not
// available as a claim and a tolerance loose enough to pretend otherwise would
// produce a green suite that measures nothing. The differences are split into
// TWO TIERS with a different sensor each, following
// rtc_controllers/test/test_dls_convergence.cpp.
//
// What moved (compute.cpp, both branches):
//   JJt = J Jᵀ; JJt.diagonal() += damping²; LDLT; JJt_inv = ldlt.solve(I);
//   Jpinv = Jᵀ·JJt_inv                         →  compliance::DifferentialIk
// and the null-space secondary task's
//   null_dq = (null_err − Jpinv·(J·null_err))·k → Solve(twist, k·null_err, out)
//
// ── Tier 1: the λ rule. A LAW CHANGE, and the point of the migration. ────────
// λ² = damping² (a constant 1e-4 at the shipped damping: 0.01) → the §6.5
// σ_min-adaptive ramp. This does NOT only bite near a singularity: away from one
// the old form damps by 1e-4 where the new one damps by EXACTLY ZERO, so the two
// laws differ at every ordinary pose. They agree at exactly one σ_min,
//
//     σ* = σ₀·√(1 − λ_c²/λ_max²)                    (≈ 0.0196 at 0.02/0.05/0.01)
//
// which is INSIDE the shell: above σ* the old law damps more, below it the new
// law does. No tolerance covers that, and one loose enough to try would make the
// tier-2 oracle vacuous. So tier 1 is not covered by tolerance — it is pinned as
// a law, by its properties (zero outside the shell, monotone into it, bounded by
// λ_max²) and by the crossover above.
//
// ── Tier 2: structure. Rounding, and nothing else. ───────────────────────────
// LDLT of a materialised inverse → LLT of a solve, and fixed size (6×6 / 3×3) →
// dynamic; plus, in the null lane, the gain moving from after the projection to
// before it and N being materialised rather than avoided. The oracle is the
// literal pre-#282 block, kept verbatim except that λ² is passed IN — so both
// sides damp identically and tier 1 cannot leak into the tier-2 budget.
//
// The bound is MEASURED HERE and is deliberately not the ≤1.7e-12 that
// differential_ik.hpp's migration note carries: that number was CLIK's, whose
// inline was already an LLT, and this one is an LDLT of a materialised inverse.
// A different decomposition is a different rounding profile, so the old number
// would have been an inherited guess. Measured worst relative difference over
// the fixtures below (iiwa7 σ_min ∈ [0, 0.206] — the fully extended pose is
// exactly singular — plus the prescribed-σ_min family across the shell):
//
//     6-DOF task lane   4.1578e-13
//     3-DOF task lane   2.5649e-14
//     null-space lane   2.3448e-14      (gain reordered across the projector)
//
// kTier2Bound is 1e-9, ~2400× the worst of those. The three tests re-record
// their own figure as a `worst_relative_difference` property, so a later change
// that erodes the margin is visible in the XML without re-deriving it.
//
// What keeps 1e-9 a real bound rather than a shrug is
// RelativeBoundRejectsTheConstantLambdaLaw, which feeds the oracle the OLD
// constant λ² and shows the same comparison fails — simultaneously the
// non-vacuity partner and the demonstration that the two tiers had to be split.
//
// The binding-level half of this migration (that demo_task_controller actually
// CALLS this law, floors λ_max at the point of use, and holds on a non-finite
// Jacobian) lives in test_demo_task_controller.cpp, which owns the controller
// fixture.

// no_malloc_scope.hpp MUST precede every Eigen header — it installs the Eigen
// allocation tripwire by defining EIGEN_RUNTIME_NO_MALLOC and its own
// eigen_assert, and a later include would be silently ignored (its own #error
// enforces the ordering). Everything below pulls Eigen, so it comes first.
// clang-format off — this include must stay FIRST. The repo's IncludeBlocks
// setting merges and re-sorts adjacent blocks, which puts the fixture header
// (and the Eigen it pulls) above this one and trips the #error below.
#include "rtc_base/testing/no_malloc_scope.hpp"
// clang-format on

#include "iiwa7_leap_test_fixture.hpp"
#include "rtc_controllers/compliance/differential_ik.hpp"
#include "rtc_controllers/compliance/task_dynamics.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace {

namespace compliance = rtc::compliance;

// The shipped contract these fixtures reproduce: config/*/demo_task_controller
// carries singularity_threshold 0.02 / max_damping 0.05 after #282, and carried
// damping 0.01 before it.
constexpr double kSigma0 = 0.02;
constexpr double kLambdaMax = 0.05;
constexpr double kRetiredLambda = 0.01;  ///< the constant λ this migration removed

// Tier-2 budget. Three orders of magnitude above the worst structural difference
// measured over the fixtures below (see the header comment), so ordinary
// rounding passes, and — as the non-vacuity partner shows — many orders below
// the λ-rule change, so a law change cannot hide inside it.
constexpr double kTier2Bound = 1e-9;

constexpr int kNv = integrated_bringup::testfx::kArmDof;  // iiwa7: 7

// ── The tier-2 oracle: the pre-#282 inline block, verbatim except for λ² ─────
// Copied from compute.cpp as it stood at main 41602f66, keeping the LDLT, the
// materialised inverse and the fixed-size m×m type. The ONLY edit is that
// `gains.damping * gains.damping` became a parameter, which is what lets this
// isolate structure from the law.
template <int M>
class InlineDlsOracle {
 public:
  using MatM = Eigen::Matrix<double, M, M>;

  void Compute(const Eigen::MatrixXd& J, double lambda_sq) {
    JJt_.noalias() = J * J.transpose();
    JJt_.diagonal().array() += lambda_sq;
    ldlt_.compute(JJt_);
    JJt_inv_.noalias() = ldlt_.solve(MatM::Identity());
    Jpinv_.noalias() = J.transpose() * JJt_inv_;
  }

  [[nodiscard]] const Eigen::MatrixXd& Jpinv() const noexcept { return Jpinv_; }

  /// The pre-#282 null-space spelling: the projector is never materialised and
  /// the gain multiplies the PROJECTED result.
  [[nodiscard]] Eigen::VectorXd NullTerm(const Eigen::MatrixXd& J, const Eigen::VectorXd& null_err,
                                         double k) const {
    Eigen::VectorXd nd = Jpinv_ * (J * null_err);
    nd = null_err - nd;
    nd *= k;
    return nd;
  }

 private:
  MatM JJt_;
  MatM JJt_inv_;
  Eigen::MatrixXd Jpinv_;
  Eigen::LDLT<MatM> ldlt_;
};

// std::to_string on a double is %f — every tier-2 figure of merit is smaller
// than its six decimals and would be recorded as "0.000000", i.e. as no
// measurement at all. These numbers are the provenance for kTier2Bound, so they
// have to survive being written down.
[[nodiscard]] std::string Sci(double v) {
  std::array<char, 32> buf{};
  std::snprintf(buf.data(), buf.size(), "%.4e", v);
  return std::string(buf.data());
}

[[nodiscard]] double RelDiff(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
  const double scale = std::max({a.norm(), b.norm(), 1e-300});
  return (a - b).norm() / scale;
}

// ── Fixture A: the Jacobians the shipped binding actually forms ──────────────
// iiwa7 is one of the three wired robots and the one the rest of this package's
// URDF-backed tests already build, so the tier-2 number is measured on real
// robot conditioning rather than on random matrices.
// Returned by reference: the static outlives every caller, and returning it by
// value deep-copied 24 dynamically-sized 6×7 matrices on each of the six call
// sites for nothing.
[[nodiscard]] const std::vector<Eigen::MatrixXd>& RealArmJacobians() {
  static const std::vector<Eigen::MatrixXd> kJs = [] {
    auto builder = integrated_bringup::testfx::SharedIiwa7LeapBuilder();
    rtc_urdf_bridge::RtModelHandle handle(builder->GetReducedModel("iiwa7"));
    const auto fid = handle.GetFrameId("ee_link");

    std::vector<Eigen::MatrixXd> js;
    std::mt19937 rng(20260729u);
    std::uniform_real_distribution<double> jitter(-1.2, 1.2);

    std::vector<double> q(static_cast<std::size_t>(kNv), 0.0);
    for (int s = 0; s < 24; ++s) {
      for (int i = 0; i < kNv; ++i) {
        // s == 0 is the fixture home pose; the rest walk the workspace, and the
        // all-zero pose (s == 1) is iiwa7 fully extended — the near-singular end.
        q[static_cast<std::size_t>(i)] =
            s == 0 ? integrated_bringup::testfx::kArmHome[static_cast<std::size_t>(i)]
                   : (s == 1 ? 0.0 : jitter(rng));
      }
      handle.ComputeJacobians(std::span<const double>(q.data(), q.size()));
      Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, kNv);
      handle.GetFrameJacobian(fid, pinocchio::LOCAL_WORLD_ALIGNED, J);
      js.push_back(J);
    }
    return js;
  }();
  return kJs;
}

// ── Fixture B: Jacobians with a PRESCRIBED σ_min ─────────────────────────────
// Walking to a specific σ_min on a real arm means solving an inverse problem;
// composing one from an SVD does not. Tier 1's claims are about specific points
// on the ramp (σ₀, σ*, deep inside), so those points have to be reachable
// exactly rather than approached.
[[nodiscard]] Eigen::MatrixXd JacobianWithSigmaMin(double sigma_min, int m, unsigned seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> gauss(0.0, 1.0);
  Eigen::MatrixXd A(m, kNv);
  for (int r = 0; r < m; ++r) {
    for (int c = 0; c < kNv; ++c) {
      A(r, c) = gauss(rng);
    }
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd s(m);
  // Descending, smallest exactly sigma_min; the rest an order apart so σ_min is
  // unambiguous rather than one of a clustered pair.
  for (int i = 0; i < m; ++i) {
    s(i) = sigma_min * std::pow(3.0, static_cast<double>(m - 1 - i));
  }
  return svd.matrixU() * s.asDiagonal() * svd.matrixV().transpose();
}

// ── Fixture coverage ────────────────────────────────────────────────────────
// A fixture whose every pose sits far outside the shell would leave the whole
// suite green through any change to where the shell begins — the tier-2 lanes
// would only ever compare λ² = 0, and the tier-1 crossover would be the only
// thing still measuring anything. So the span is asserted rather than assumed,
// and it names WHICH fixture supplies each side.
TEST(FixtureCoverage, SpansBothSidesOfTheSingularityShell) {
  compliance::DifferentialIk ik;
  ik.Resize(kNv, 6);

  int real_outside = 0;
  double real_min = 1e9;
  double real_max = 0.0;
  for (const auto& J : RealArmJacobians()) {
    const auto r = ik.Compute(J, kSigma0, kLambdaMax);
    ASSERT_TRUE(r.ok);
    real_min = std::min(real_min, r.sigma_min);
    real_max = std::max(real_max, r.sigma_min);
    if (r.sigma_min >= kSigma0) {
      ++real_outside;
    }
  }
  RecordProperty("real_sigma_min_range", Sci(real_min) + " .. " + Sci(real_max));
  EXPECT_GE(real_outside, 10) << "the real-arm fixture must supply well-conditioned poses";

  // The in-shell side is the prescribed-σ family's job, and it must actually
  // produce a non-zero λ² — otherwise "covers the shell" is a claim about a
  // constructor, not about what the law saw.
  int inside_with_damping = 0;
  for (const double frac : {0.5, 0.1, 0.01}) {
    const Eigen::MatrixXd J = JacobianWithSigmaMin(kSigma0 * frac, 6, 61u);
    const auto r = ik.Compute(J, kSigma0, kLambdaMax);
    ASSERT_TRUE(r.ok);
    EXPECT_LT(r.sigma_min, kSigma0);
    if (r.lambda_sq > 0.0) {
      ++inside_with_damping;
    }
  }
  EXPECT_EQ(inside_with_damping, 3) << "the in-shell fixture must reach λ² > 0";
}

// ── Tier 1: the λ law ───────────────────────────────────────────────────────

// Outside the shell the new law damps by EXACTLY zero — not "a little", which is
// what makes this a law change rather than a retuning. Asserted through
// DifferentialIk (what the binding calls), not only through the free function.
TEST(Tier1AdaptiveLambdaLaw, IsExactlyZeroOutsideTheShell) {
  compliance::DifferentialIk ik;
  ik.Resize(kNv, 6);
  for (const double sigma : {kSigma0, kSigma0 * 1.5, 0.1, 0.5, 1.0}) {
    const Eigen::MatrixXd J = JacobianWithSigmaMin(sigma, 6, 11u);
    const auto r = ik.Compute(J, kSigma0, kLambdaMax);
    ASSERT_TRUE(r.ok);
    EXPECT_NEAR(r.sigma_min, sigma, 1e-12 * std::max(1.0, sigma));
    EXPECT_EQ(r.lambda_sq, 0.0) << "σ_min = " << sigma
                                << " ≥ σ₀ must damp by exactly 0; the "
                                   "retired law damped by "
                                << kRetiredLambda * kRetiredLambda << " here";
  }
}

// Into the shell: strictly monotone and bounded by λ_max². The bound is the
// property AdaptiveDampingSquared owns (NUM-1) and the reason the floor lives in
// the CONSUMER rather than inside the law.
TEST(Tier1AdaptiveLambdaLaw, IsMonotoneIntoTheShellAndBoundedByLambdaMaxSquared) {
  compliance::DifferentialIk ik;
  ik.Resize(kNv, 6);
  double prev = -1.0;
  // Descending σ_min ⇒ ascending λ².
  for (const double frac : {0.99, 0.75, 0.5, 0.25, 0.1, 0.01, 0.0}) {
    const double sigma = kSigma0 * frac;
    const Eigen::MatrixXd J = JacobianWithSigmaMin(std::max(sigma, 1e-14), 6, 12u);
    const auto r = ik.Compute(J, kSigma0, kLambdaMax);
    ASSERT_TRUE(r.ok);
    EXPECT_GT(r.lambda_sq, prev) << "λ² must grow as σ_min → 0 (σ_min = " << sigma << ")";
    EXPECT_LE(r.lambda_sq, kLambdaMax * kLambdaMax)
        << "λ² ≤ λ_max² is the law's own named property";
    prev = r.lambda_sq;
  }
}

// The whole reason the two tiers exist: the laws coincide at ONE σ_min, and it
// is inside the shell. Above it the retired constant damped MORE, below it the
// ramp does — so no single tolerance can describe the migration.
TEST(Tier1AdaptiveLambdaLaw, TheTwoLawsAgreeOnlyAtTheCrossoverShell) {
  const double lambda_c_sq = kRetiredLambda * kRetiredLambda;
  const double sigma_star = kSigma0 * std::sqrt(1.0 - lambda_c_sq / (kLambdaMax * kLambdaMax));
  ASSERT_GT(sigma_star, 0.0);
  ASSERT_LT(sigma_star, kSigma0) << "the crossover must sit INSIDE the shell";

  EXPECT_NEAR(compliance::AdaptiveDampingSquared(sigma_star, kSigma0, kLambdaMax), lambda_c_sq,
              1e-15);
  EXPECT_LT(compliance::AdaptiveDampingSquared(sigma_star * 1.05, kSigma0, kLambdaMax), lambda_c_sq)
      << "above σ* the RETIRED constant law was the more damped of the two";
  EXPECT_GT(compliance::AdaptiveDampingSquared(sigma_star * 0.95, kSigma0, kLambdaMax), lambda_c_sq)
      << "below σ* the adaptive ramp is the more damped of the two";
}

// Tier 1 in the units that ship: the joint velocity the two laws command from
// the same Jacobian and the same task velocity, at poses the robot actually
// visits. The point is that this is O(1e-3) relative — far above kTier2Bound —
// at ORDINARY poses, not only near a singularity.
TEST(Tier1AdaptiveLambdaLaw, DivergesFromTheConstantLawAtOrdinaryPoses) {
  compliance::DifferentialIk ik;
  ik.Resize(kNv, 6);
  InlineDlsOracle<6> retired;

  Eigen::Matrix<double, 6, 1> task_vel;
  task_vel << 0.05, -0.03, 0.02, 0.01, -0.02, 0.015;
  Eigen::VectorXd dq_new(kNv);

  int ordinary = 0;
  double worst = 0.0;
  for (const auto& J : RealArmJacobians()) {
    const auto r = ik.Compute(J, kSigma0, kLambdaMax);
    ASSERT_TRUE(r.ok);
    if (r.sigma_min < kSigma0) {
      continue;  // inside the shell — covered by the crossover test above
    }
    ++ordinary;
    ik.Solve(task_vel, dq_new);
    retired.Compute(J, kRetiredLambda * kRetiredLambda);
    const Eigen::VectorXd dq_retired = retired.Jpinv() * task_vel;
    const double rel = RelDiff(dq_new, dq_retired);
    worst = std::max(worst, rel);
    EXPECT_GT(rel, kTier2Bound * 1e3)
        << "at σ_min = " << r.sigma_min
        << " the constant-λ and adaptive laws must differ by far more than rounding — otherwise "
           "the two tiers would not need separating";
  }
  ASSERT_GE(ordinary, 10) << "fixture drifted: too few well-conditioned poses to make the claim";
  EXPECT_LT(worst, 1.0) << "sanity: the law change is a perturbation, not a different controller";
}

// ── Tier 2: structure, at matched λ² ─────────────────────────────────────────

// Both task lanes. λ² is read back from the new law and handed to the oracle, so
// the only thing left between them is LDLT-of-materialised-inverse vs
// LLT-of-solve at fixed vs dynamic size.
template <int M>
void ExpectTaskLaneMatchesOracle(double* worst_out) {
  compliance::DifferentialIk ik;
  ik.Resize(kNv, M);
  InlineDlsOracle<M> oracle;

  Eigen::Matrix<double, M, 1> task_vel;
  for (int i = 0; i < M; ++i) {
    task_vel(i) = 0.05 - 0.017 * static_cast<double>(i);
  }
  Eigen::VectorXd dq_new(kNv);

  std::vector<Eigen::MatrixXd> js;
  for (const auto& J6 : RealArmJacobians()) {
    js.push_back(M == 6 ? J6 : Eigen::MatrixXd(J6.topRows(M)));
  }
  // Plus the prescribed-σ_min family, so the shell (where λ² ≠ 0 and the two
  // decompositions have the most to disagree about) is covered too.
  for (const double frac : {2.0, 1.0, 0.5, 0.1, 0.01}) {
    js.push_back(JacobianWithSigmaMin(kSigma0 * frac, M, 21u + static_cast<unsigned>(frac * 100)));
  }

  for (const auto& J : js) {
    const auto r = ik.Compute(J, kSigma0, kLambdaMax);
    ASSERT_TRUE(r.ok);
    ik.Solve(task_vel, dq_new);
    oracle.Compute(J, r.lambda_sq);  // ← matched λ²: tier 1 cannot leak in here
    const Eigen::VectorXd dq_oracle = oracle.Jpinv() * task_vel;
    const double rel = RelDiff(dq_new, dq_oracle);
    *worst_out = std::max(*worst_out, rel);
    EXPECT_LE(rel, kTier2Bound) << "m = " << M << ", σ_min = " << r.sigma_min;
  }
}

TEST(Tier2Structure, SixDofTaskLaneMatchesTheInlineOracle) {
  double worst = 0.0;
  ExpectTaskLaneMatchesOracle<6>(&worst);
  RecordProperty("worst_relative_difference", Sci(worst));
}

TEST(Tier2Structure, ThreeDofTaskLaneMatchesTheInlineOracle) {
  double worst = 0.0;
  ExpectTaskLaneMatchesOracle<3>(&worst);
  RecordProperty("worst_relative_difference", Sci(worst));
}

// The null lane carries two structural changes the task lanes do not: the gain
// moved from after the projection to before it, and N is now materialised
// instead of being avoided via null_err − J⁺(J·null_err). Measured separately
// because a bound that held for the task lane says nothing about this one.
TEST(Tier2Structure, NullSpaceLaneMatchesTheInlineOracle) {
  compliance::DifferentialIk ik;
  ik.Resize(kNv, 3);
  InlineDlsOracle<3> oracle;

  const Eigen::Vector3d task_vel(0.05, -0.03, 0.02);
  const double null_kp = 0.5;  // the shipped default
  Eigen::VectorXd null_err(kNv);
  for (int i = 0; i < kNv; ++i) {
    null_err(i) = 0.4 - 0.13 * static_cast<double>(i);
  }

  Eigen::VectorXd dq_new(kNv);
  Eigen::VectorXd qdot_null = null_kp * null_err;
  double worst = 0.0;

  std::vector<Eigen::MatrixXd> js;
  for (const auto& J6 : RealArmJacobians()) {
    js.push_back(J6.topRows(3));
  }
  // The prescribed-σ family too: without it this lane would only ever see
  // λ² = 0, and the in-shell rounding — where the two decompositions have the
  // most to disagree about — would go unmeasured while the test stayed green.
  for (const double frac : {2.0, 1.0, 0.5, 0.1, 0.01}) {
    js.push_back(JacobianWithSigmaMin(kSigma0 * frac, 3, 41u + static_cast<unsigned>(frac * 100)));
  }

  for (const auto& J : js) {
    const auto r = ik.Compute(J, kSigma0, kLambdaMax);
    ASSERT_TRUE(r.ok);
    ik.Solve(task_vel, qdot_null, dq_new);

    oracle.Compute(J, r.lambda_sq);
    const Eigen::VectorXd dq_oracle =
        oracle.Jpinv() * task_vel + oracle.NullTerm(J, null_err, null_kp);

    const double rel = RelDiff(dq_new, dq_oracle);
    worst = std::max(worst, rel);
    EXPECT_LE(rel, kTier2Bound) << "null lane, σ_min = " << r.sigma_min;
  }
  RecordProperty("worst_relative_difference", Sci(worst));
}

// ── Non-vacuity: the bound rejects the law change ────────────────────────────
// Without this, kTier2Bound could be any number at all and the three tests above
// would still be green. Feeding the oracle the RETIRED constant λ² — the one
// difference tier 2 is defined to exclude — must break the same comparison.
TEST(Tier2Structure, RelativeBoundRejectsTheConstantLambdaLaw) {
  compliance::DifferentialIk ik;
  ik.Resize(kNv, 6);
  InlineDlsOracle<6> oracle;

  Eigen::Matrix<double, 6, 1> task_vel;
  task_vel << 0.05, -0.03, 0.02, 0.01, -0.02, 0.015;
  Eigen::VectorXd dq_new(kNv);

  int rejected = 0;
  int considered = 0;
  for (const auto& J : RealArmJacobians()) {
    const auto r = ik.Compute(J, kSigma0, kLambdaMax);
    ASSERT_TRUE(r.ok);
    ik.Solve(task_vel, dq_new);
    oracle.Compute(J, kRetiredLambda * kRetiredLambda);  // ← the OLD λ², on purpose
    ++considered;
    if (RelDiff(dq_new, oracle.Jpinv() * task_vel) > kTier2Bound) {
      ++rejected;
    }
  }
  EXPECT_EQ(rejected, considered)
      << "kTier2Bound must reject the constant-λ law at EVERY fixture pose; if it does not, the "
         "tier-2 tests are measuring nothing and a law change could pass as rounding";
}

// ── RT: the tick allocates nothing ──────────────────────────────────────────
// Both gates, because they see different things: ScopedAllocGate counts
// operator new, ScopedNoMalloc rides eigen_assert and catches Eigen's own
// internal allocations. Resize() is the allocating entry point and is called
// OUTSIDE the gate, mirroring InitArmModel vs ComputeControl.
TEST(RtSafety, ComputeAndSolveAreHeapFreeForBothTaskDimensions) {
  compliance::DifferentialIk ik6;
  compliance::DifferentialIk ik3;
  ik6.Resize(kNv, 6);
  ik3.Resize(kNv, 3);

  const Eigen::MatrixXd J6 = RealArmJacobians().front();
  const Eigen::MatrixXd J3 = J6.topRows(3);

  Eigen::Matrix<double, 6, 1> tw6;
  tw6 << 0.05, -0.03, 0.02, 0.01, -0.02, 0.015;
  const Eigen::Vector3d tw3(0.05, -0.03, 0.02);
  Eigen::VectorXd dq(kNv);
  Eigen::VectorXd qdot_null = Eigen::VectorXd::Constant(kNv, 0.2);

  // Warm the LLT/eigensolver work paths once before arming, so a first-call
  // lazy allocation inside Eigen is attributed rather than hidden.
  (void)ik6.Compute(J6, kSigma0, kLambdaMax);
  (void)ik3.Compute(J3, kSigma0, kLambdaMax);
  ik6.Solve(tw6, dq);
  ik3.Solve(tw3, qdot_null, dq);

  std::size_t new_count = 0;
  std::uint64_t eigen_count = 0;
  {
    const rtc::testing::ScopedAllocGate new_gate;
    const rtc::testing::ScopedNoMalloc eigen_gate;
    for (int i = 0; i < 8; ++i) {
      (void)ik6.Compute(J6, kSigma0, kLambdaMax);
      ik6.Solve(tw6, dq);
      (void)ik3.Compute(J3, kSigma0, kLambdaMax);
      ik3.Solve(tw3, qdot_null, dq);
    }
    new_count = new_gate.count();
    eigen_count = eigen_gate.violations();
  }
  EXPECT_EQ(new_count, 0u) << "operator new on the RT tick (RT-1)";
  EXPECT_EQ(eigen_count, 0u) << "Eigen internal allocation on the RT tick (RT-1)";
}

}  // namespace
