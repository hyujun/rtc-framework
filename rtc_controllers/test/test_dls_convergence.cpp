// ── Convergence harness for the two constant-λ DLS outliers (#236 S2b+S3b) ───
// Sixth and seventh slices of the rtc_controllers pure-library refactor, and the
// ONLY ones where "the extraction changed NOTHING" is not the claim. Every
// earlier slice compared bit for bit (rtc::testing::BitsEqual). This one cannot,
// and pretending otherwise would produce a green suite that measures nothing —
// so the differences are split into TWO TIERS with a different sensor each.
//
// What moved:
//   • OperationalSpaceController's inline Λ⁻¹ = J M⁻¹ Jᵀ + λ²I / F = Λ a_task /
//     Nᵀ = I − Jᵀ J̄ᵀ block  →  compliance::TaskDynamics.
//   • ClikController's two inline damped-pseudoinverse branches  →
//     compliance::DifferentialIk (issue #258).
// Both adapters were the last CONSTANT-λ spellings in the repo; the other three
// task-space controllers already called these helpers.
//
// ── Tier 1: the λ rule. A LAW CHANGE, and the point of the migration. ────────
// λ = max(1e-4, damping) → the §6.5 σ_min-adaptive ramp. Away from a singularity
// the old form damped by damping² (0.0025 at the default 0.05) where the new one
// damps by EXACTLY ZERO. That is O(1) at every ordinary pose; no tolerance
// covers it, and a tolerance loose enough to try would make the tier-2 oracle
// vacuous. So it is not covered by tolerance — it is pinned as a law, by its
// properties (zero outside the shell, monotone into it, bounded by λ_max²), and
// the adapters are pinned to be USING that law rather than any other.
//
// ── Tier 2: structure. Rounding, and nothing else. ───────────────────────────
// LDLT → LLT, materialised inverse → solve, fixed size → dynamic, and (CLIK's
// posture task) the gain moving from after the projection to before it. Probes
// measured every one of these at ≤ 1.7e-12 relative. The oracle here is the
// literal pre-extraction block, kept verbatim except that λ² is passed IN — so
// both sides damp identically and tier 1 cannot leak into the tier-2 budget.
// That is what makes 1e-9 a real bound rather than a shrug:
// RelativeBoundRejectsTheConstantLambdaLaw feeds the oracle the OLD constant λ²
// and shows the same comparison fails, which is simultaneously the non-vacuity
// partner and the demonstration that the two tiers had to be separated.
//
// ── Why a non-bit-identical migration was acceptable at all ─────────────────
// Neither adapter is instantiated outside rtc_controllers: integrated_bringup
// registers demo_joint/task/wbc_controller and nothing else, so runtime exposure
// is zero (#236 R1/D-S2b). The wired sixth copy of this law —
// demo_task_controller's own inline DLS — is deliberately NOT touched here; it
// is a binding, and S7 owns it.

// no_malloc_scope.hpp MUST precede every Eigen header — it installs the Eigen
// allocation tripwire by defining EIGEN_RUNTIME_NO_MALLOC and its own
// eigen_assert, and a later include would be silently ignored (its own #error
// enforces the ordering). Everything below pulls Eigen, so it comes first.
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_controllers/compliance/differential_ik.hpp"
#include "rtc_controllers/compliance/task_dynamics.hpp"
#include "rtc_controllers/joint/posture_law.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/bit_compare.hpp"
#include "rtc_controllers/testing/serial7dof_fixture.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace {

using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

using rtc::testing::MakeHandle;
using rtc::testing::MakeRng;

namespace compliance = rtc::compliance;

// Tier-2 budget. Three orders of magnitude above the worst measured structural
// difference (1.7e-12) so ordinary rounding passes, and — as the non-vacuity
// partner shows — many orders below the λ-rule change, so a law change cannot
// hide underneath it. Anything between is not rounding; it is a defect.
constexpr double kStructuralTolerance = 1e-9;

// Per-component difference scaled by the LANE's magnitude, not by the
// component's. A joint torque or velocity lane routinely has a near-zero
// component produced by cancellation between O(10) terms; dividing by that
// component would report a huge "relative" error for a last-bit difference and
// make the bound untestable. Scaling by ‖·‖_∞ (floored at 1) asks the question
// that matters: is the disagreement small compared to what this lane carries?
//
// The size check must be FATAL, and it cannot be ASSERT_EQ because this returns
// a double. A non-fatal EXPECT records the failure and then falls through into
// `a - b`, whose only size check is an eigen_assert — compiled out under NDEBUG,
// which is this repo's default build (design-principles.md §코어의 형태). The
// subtraction would then read past the shorter buffer and report adjacent memory
// as the "worst scaled difference". Returning infinity makes every downstream
// tolerance comparison fail on purpose, so a mis-sized oracle is loud.
[[nodiscard]] double MaxScaledDiff(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
  if (a.size() != b.size()) {
    ADD_FAILURE() << "MaxScaledDiff operands differ in size (" << a.size() << " vs " << b.size()
                  << ") — the comparison below would read out of bounds under NDEBUG";
    return std::numeric_limits<double>::infinity();
  }
  const double scale = std::max({1.0, a.cwiseAbs().maxCoeff(), b.cwiseAbs().maxCoeff()});
  return (a - b).cwiseAbs().maxCoeff() / scale;
}

// ═══════════════════════════════════════════════════════════════════════════
// Model fixture — the shared serial_7dof (nv = 7 > 6, so Nᵀ is non-trivial)
// ═══════════════════════════════════════════════════════════════════════════

struct Snapshot {
  Eigen::MatrixXd J_full;  ///< 6×nv
  Eigen::MatrixXd J_pos;   ///< 3×nv
  Eigen::MatrixXd M;       ///< nv×nv, symmetrised
  Eigen::VectorXd h;       ///< nv
  Eigen::VectorXd q;       ///< nv
  Eigen::VectorXd v;       ///< nv
};

class ModelRig {
 public:
  ModelRig() {
    handle_ = MakeHandle(model_);
    tip_ = static_cast<pinocchio::FrameIndex>(model_->nframes - 1);
    nv_ = handle_->nv();
  }

  [[nodiscard]] int nv() const { return nv_; }

  /// Evaluate J, M and h at a joint state. `q`/`v` are nv-long.
  [[nodiscard]] Snapshot At(const std::vector<double>& q, const std::vector<double>& v) {
    Snapshot s;
    const auto n = static_cast<std::size_t>(nv_);
    const std::span<const double> q_span(q.data(), n);
    const std::span<const double> v_span(v.data(), n);

    s.J_full = Eigen::MatrixXd::Zero(6, nv_);
    handle_->ComputeJacobians(q_span);
    handle_->GetFrameJacobian(tip_, pinocchio::LOCAL_WORLD_ALIGNED, s.J_full);
    s.J_pos = s.J_full.topRows(3);

    handle_->ComputeMassMatrix(q_span);
    s.M = handle_->GetMassMatrix();
    s.M.triangularView<Eigen::StrictlyLower>() =
        s.M.triangularView<Eigen::StrictlyUpper>().transpose();

    handle_->ComputeNonLinearEffects(q_span, v_span);
    s.h = handle_->GetNonLinearEffects();

    s.q = Eigen::Map<const Eigen::VectorXd>(q.data(), nv_);
    s.v = Eigen::Map<const Eigen::VectorXd>(v.data(), nv_);
    return s;
  }

 private:
  std::shared_ptr<const pinocchio::Model> model_;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle_;
  pinocchio::FrameIndex tip_{0};
  int nv_{0};
};

/// A well-conditioned draw: joints spread away from the all-parallel-axis
/// alignment, so σ_min(J) sits comfortably above any sane σ₀.
void FillRegularPose(std::vector<double>& q, std::vector<double>& v, int nv, int k) {
  for (int j = 0; j < nv; ++j) {
    const auto uj = static_cast<std::size_t>(j);
    q[uj] = 0.35 * (1.0 + 0.21 * j) + 0.011 * k * (1.0 + 0.17 * j);
    v[uj] = 0.05 * (1.0 + 0.3 * j) - 0.004 * k;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// Oracle 1 — literal pre-extraction OSC Λ/τ/Nᵀ block
// ═══════════════════════════════════════════════════════════════════════════

// EXACTLY the block as it stood inline in OperationalSpaceController::Compute()
// before #236 S2b lifted it onto compliance::TaskDynamics
// (operational_space_controller.cpp:407-469 at 2fd5fd42). Rename-only
// transcription of the members to locals — same operation order, same
// association, same FIXED 6×6 LLT, same solveInPlace rather than a materialised
// Λ, same posture loop.
//
// ONE substantive edit, and it is the whole design of this oracle: λ² arrives as
// a PARAMETER where the inline form computed `const double lambda =
// std::max(1e-4, gains.damping); ... += lambda * lambda`. Passing it in is what
// isolates tier 2 — with the same λ² on both sides, anything the comparison sees
// is structure. Feed it the old constant and it becomes the tier-1 witness
// instead; RelativeBoundRejectsTheConstantLambdaLaw does exactly that.
struct OscInline {
  Eigen::VectorXd tau;
  bool dyn_ok{false};
};

OscInline PreExtractionOscBlock(const Snapshot& s, const Vec6& a_task, double lambda_sq,
                                double null_kp, double null_kd, const Eigen::VectorXd& q_ref,
                                int nv) {
  OscInline out;
  out.tau = Eigen::VectorXd::Zero(nv);

  Eigen::LLT<Eigen::MatrixXd> llt_M(nv);
  Eigen::MatrixXd Jt(nv, 6);
  Eigen::MatrixXd MinvJt(nv, 6);
  Mat6 LambdaInv = Mat6::Zero();
  Eigen::LLT<Mat6> llt6;
  Eigen::MatrixXd JbarT(6, nv);
  Eigen::MatrixXd NT(nv, nv);
  Eigen::VectorXd tau0 = Eigen::VectorXd::Zero(nv);
  Eigen::VectorXd null_tmp = Eigen::VectorXd::Zero(nv);

  bool dyn_ok = true;
  llt_M.compute(s.M);
  dyn_ok = dyn_ok && (llt_M.info() == Eigen::Success);

  if (dyn_ok) {
    Jt.noalias() = s.J_full.transpose();
    MinvJt = Jt;
    llt_M.solveInPlace(MinvJt);

    LambdaInv.noalias() = s.J_full * MinvJt;
    LambdaInv.diagonal().array() += lambda_sq;
    llt6.compute(LambdaInv);
    dyn_ok = (llt6.info() == Eigen::Success);
  }

  if (dyn_ok) {
    Vec6 F = a_task;
    llt6.solveInPlace(F);
    out.tau.noalias() = Jt * F;
    out.tau += s.h;

    if ((null_kp != 0.0 || null_kd != 0.0) && nv > 6) {
      JbarT = MinvJt.transpose();
      llt6.solveInPlace(JbarT);
      NT.setIdentity();
      NT.noalias() -= Jt * JbarT;
      for (int i = 0; i < nv; ++i)
        tau0[i] = null_kp * (q_ref[i] - s.q[i]) - null_kd * s.v[i];
      null_tmp.noalias() = NT * tau0;
      out.tau += null_tmp;
    }
  } else {
    out.tau = s.h;
  }
  out.dyn_ok = dyn_ok;
  return out;
}

/// The migrated form: the same tick, expressed through the shared helper. This
/// mirrors operational_space_controller.cpp's Steps 7-9 — the identity-order
/// reorders the adapter performs are omitted because they are memcpy on this
/// fixture and belong to the binding, not to the law.
///
/// #277's posture floor is deliberately NOT mirrored here, in either oracle. It
/// sits one line ABOVE Step 7 in the controller (`FloorNonNegativeGain` before the
/// gate), so it belongs to the binding the same way the reorders do — and the
/// oracle above is a frozen literal of the PRE-#236-S2b code, which cannot grow
/// a 2026 clamp without ceasing to be the thing it witnesses. Both oracles are
/// driven with non-negative gains only, where the floor is the identity, so the
/// pair still compares like for like. Feed this file a negative gain and you
/// must floor at the CALL SITE, not in here.
OscInline MigratedOscBlock(compliance::TaskDynamics& dyn, const Snapshot& s, const Vec6& a_task,
                           double sigma0, double lambda_max, double null_kp, double null_kd,
                           const Eigen::VectorXd& q_ref, int nv, double* lambda_sq_out,
                           double* sigma_min_out) {
  OscInline out;
  out.tau = Eigen::VectorXd::Zero(nv);

  Eigen::LLT<Eigen::MatrixXd> llt_M(nv);
  Eigen::MatrixXd Jt(nv, 6);
  Eigen::VectorXd tau0 = Eigen::VectorXd::Zero(nv);
  Eigen::VectorXd null_tmp = Eigen::VectorXd::Zero(nv);

  bool dyn_ok = true;
  llt_M.compute(s.M);
  dyn_ok = dyn_ok && (llt_M.info() == Eigen::Success);

  if (dyn_ok) {
    Jt.noalias() = s.J_full.transpose();
    const compliance::TaskDynamics::Result r = dyn.Compute(s.J_full, llt_M, sigma0, lambda_max);
    dyn_ok = r.ok;
    if (lambda_sq_out != nullptr)
      *lambda_sq_out = r.lambda_sq;
    if (sigma_min_out != nullptr)
      *sigma_min_out = r.sigma_min;
  }

  if (dyn_ok) {
    const Vec6 F = dyn.LambdaS() * a_task;
    out.tau.noalias() = Jt * F;
    out.tau += s.h;

    if ((null_kp != 0.0 || null_kd != 0.0) && nv > 6) {
      const auto nvu = static_cast<std::size_t>(nv);
      rtc::joint::ComputePostureTorque(
          null_kp, null_kd,
          rtc::joint::PostureInputs{{q_ref.data(), nvu}, {s.q.data(), nvu}, {s.v.data(), nvu}}, nvu,
          {tau0.data(), nvu});
      dyn.ProjectNullspace(tau0, null_tmp);
      out.tau += null_tmp;
    }
  } else {
    out.tau = s.h;
  }
  out.dyn_ok = dyn_ok;
  return out;
}

// ═══════════════════════════════════════════════════════════════════════════
// Oracle 2 — literal pre-extraction CLIK damped-pseudoinverse branches
// ═══════════════════════════════════════════════════════════════════════════

// EXACTLY the two branches as they stood inline in ClikController::Compute()
// before #236 S3b (clik_controller.cpp:423-451 at 2fd5fd42): LDLT of J Jᵀ + λ²I,
// the inverse MATERIALISED via solve(I), then J⁺ = Jᵀ·(J Jᵀ)⁻¹, all at fixed
// size. Same single edit as the OSC oracle — λ² is a parameter, not
// `gains.damping * gains.damping`.
template <int M>
Eigen::MatrixXd PreExtractionClikPseudoInverse(const Eigen::MatrixXd& J, double lambda_sq) {
  using MatM = Eigen::Matrix<double, M, M>;
  MatM JJt = MatM::Zero();
  JJt.noalias() = J * J.transpose();
  JJt.diagonal().array() += lambda_sq;
  Eigen::LDLT<MatM> ldlt;
  ldlt.compute(JJt);
  MatM JJt_inv = MatM::Zero();
  JJt_inv.noalias() = ldlt.solve(MatM::Identity());
  Eigen::MatrixXd Jpinv(J.cols(), M);
  Jpinv.noalias() = J.transpose() * JJt_inv;
  return Jpinv;
}

// ═══════════════════════════════════════════════════════════════════════════
// Tier 1 — the λ rule itself (a law, pinned as properties)
// ═══════════════════════════════════════════════════════════════════════════

TEST(AdaptiveDampingLaw, IsExactlyZeroOutsideTheSingularShell) {
  constexpr double kSigma0 = 0.02;
  constexpr double kLambdaMax = 0.05;
  // Exactly zero, not merely small: that is what preserves the exact nullspace
  // orthogonality J M⁻¹ Nᵀ = 0 away from singularities, and it is precisely what
  // the constant-λ form these adapters used could never produce.
  for (const double sigma : {0.02, 0.0200001, 0.05, 0.4, 12.0}) {
    EXPECT_TRUE(rtc::testing::BitsEqual(
        compliance::AdaptiveDampingSquared(sigma, kSigma0, kLambdaMax), 0.0))
        << "sigma_min = " << sigma;
  }
}

TEST(AdaptiveDampingLaw, GrowsMonotonicallyAsSigmaMinFallsThroughTheShell) {
  constexpr double kSigma0 = 0.02;
  constexpr double kLambdaMax = 0.05;
  double prev = -1.0;
  // Walk σ_min DOWN from the shell boundary to the singular pose.
  for (int i = 20; i >= 0; --i) {
    const double sigma = kSigma0 * static_cast<double>(i) / 20.0;
    const double lambda_sq = compliance::AdaptiveDampingSquared(sigma, kSigma0, kLambdaMax);
    EXPECT_GT(lambda_sq, prev) << "sigma_min = " << sigma;
    prev = lambda_sq;
  }
  // At σ_min = 0 the ramp is saturated: λ² = λ_max² exactly.
  EXPECT_DOUBLE_EQ(compliance::AdaptiveDampingSquared(0.0, kSigma0, kLambdaMax),
                   kLambdaMax * kLambdaMax);
}

TEST(AdaptiveDampingLaw, NeverExceedsLambdaMaxSquared) {
  constexpr double kSigma0 = 0.02;
  constexpr double kLambdaMax = 0.05;
  auto rng = MakeRng();
  std::uniform_real_distribution<double> sigma_dist(0.0, 3.0 * kSigma0);
  for (int i = 0; i < 5000; ++i) {
    const double lambda_sq =
        compliance::AdaptiveDampingSquared(sigma_dist(rng), kSigma0, kLambdaMax);
    EXPECT_GE(lambda_sq, 0.0);
    EXPECT_LE(lambda_sq, kLambdaMax * kLambdaMax);
  }
}

TEST(AdaptiveDampingLaw, NonPositiveSigmaZeroDisarmsTheRampRatherThanDividing) {
  // NUM-2: σ₀ is a denominator. A zero or negative threshold must return 0
  // rather than divide — and returning 0 is also the right SEMANTICS ("no shell
  // configured"), so the guard is not a special case bolted onto the law.
  EXPECT_TRUE(rtc::testing::BitsEqual(compliance::AdaptiveDampingSquared(0.0, 0.0, 0.05), 0.0));
  EXPECT_TRUE(rtc::testing::BitsEqual(compliance::AdaptiveDampingSquared(0.1, -1.0, 0.05), 0.0));
}

// ── Tier 1 wiring: the migrated blocks USE that rule, not some other one ─────

TEST(AdaptiveDampingLaw, MigratedOscDampsZeroWhenWellConditionedAndRampsNearSingularity) {
  ModelRig rig;
  const int nv = rig.nv();
  compliance::TaskDynamics dyn;
  dyn.Resize(nv, 6);

  std::vector<double> q(static_cast<std::size_t>(nv), 0.0);
  std::vector<double> v(static_cast<std::size_t>(nv), 0.0);
  const Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(nv);
  const Vec6 a_task = Vec6::Constant(0.3);

  // Well-conditioned: σ_min ≫ σ₀ ⇒ λ² is EXACTLY zero. The pre-migration form
  // would have damped by 0.05² = 0.0025 here, on every single tick.
  FillRegularPose(q, v, nv, 0);
  double lambda_sq = -1.0;
  double sigma_min = -1.0;
  MigratedOscBlock(dyn, rig.At(q, v), a_task, 0.02, 0.05, 0.0, 0.0, q_ref, nv, &lambda_sq,
                   &sigma_min);
  EXPECT_GT(sigma_min, 0.02) << "fixture is not well-conditioned; the claim below is vacuous";
  EXPECT_TRUE(rtc::testing::BitsEqual(lambda_sq, 0.0));

  // Rank-deficient: serial_7dof at all-zero q puts four joint axes parallel, so
  // σ_min collapses (measured, not assumed — the EXPECT_LT below is the check).
  std::fill(q.begin(), q.end(), 0.0);
  std::fill(v.begin(), v.end(), 0.0);
  MigratedOscBlock(dyn, rig.At(q, v), a_task, 0.02, 0.05, 0.0, 0.0, q_ref, nv, &lambda_sq,
                   &sigma_min);
  EXPECT_LT(sigma_min, 0.02) << "the singular fixture is not inside the shell";
  EXPECT_GT(lambda_sq, 0.0);
  EXPECT_LE(lambda_sq, 0.05 * 0.05);
}

TEST(AdaptiveDampingLaw, MigratedClikDampsZeroWhenWellConditionedAndRampsNearSingularity) {
  ModelRig rig;
  const int nv = rig.nv();
  compliance::DifferentialIk ik;
  ik.Resize(nv, 3);

  std::vector<double> q(static_cast<std::size_t>(nv), 0.0);
  std::vector<double> v(static_cast<std::size_t>(nv), 0.0);

  FillRegularPose(q, v, nv, 0);
  compliance::DifferentialIk::Result r = ik.Compute(rig.At(q, v).J_pos, 0.02, 0.05);
  ASSERT_TRUE(r.ok);
  EXPECT_GT(r.sigma_min, 0.02) << "fixture is not well-conditioned; the claim below is vacuous";
  EXPECT_TRUE(rtc::testing::BitsEqual(r.lambda_sq, 0.0));

  // A 3×nv translational Jacobian at all-zero q: the translational rows lose
  // rank as well, so the shell is genuinely entered.
  std::fill(q.begin(), q.end(), 0.0);
  r = ik.Compute(rig.At(q, v).J_pos, 0.02, 0.05);
  ASSERT_TRUE(r.ok);
  EXPECT_LT(r.sigma_min, 0.02) << "the singular fixture is not inside the shell";
  EXPECT_GT(r.lambda_sq, 0.0);
  EXPECT_LE(r.lambda_sq, 0.05 * 0.05);
}

// ═══════════════════════════════════════════════════════════════════════════
// Tier 2 — structure only, λ² matched on both sides
// ═══════════════════════════════════════════════════════════════════════════

TEST(OscTaskDynamics, MatchesThePreExtractionInlineFormWithinRounding) {
  ModelRig rig;
  const int nv = rig.nv();
  compliance::TaskDynamics dyn;
  dyn.Resize(nv, 6);

  std::vector<double> q(static_cast<std::size_t>(nv), 0.0);
  std::vector<double> v(static_cast<std::size_t>(nv), 0.0);
  Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(nv);
  for (int i = 0; i < nv; ++i)
    q_ref[i] = 0.1 * (1.0 + 0.4 * i);

  double worst = 0.0;
  int posture_ticks = 0;
  for (int k = 0; k < 60; ++k) {
    FillRegularPose(q, v, nv, k);
    const Snapshot s = rig.At(q, v);
    Vec6 a_task;
    for (int i = 0; i < 6; ++i)
      a_task[i] = 0.4 * (1.0 + 0.13 * i) - 0.007 * k;

    // Both posture-gate states, so the Nᵀ path is compared and not merely the
    // primary task (a gate closed for every draw pins nothing — S4 (0b)).
    const bool posture = (k % 2 == 0);
    const double null_kp = posture ? 4.0 : 0.0;
    const double null_kd = posture ? 0.7 : 0.0;
    posture_ticks += posture ? 1 : 0;

    double lambda_sq = 0.0;
    const OscInline migrated = MigratedOscBlock(dyn, s, a_task, 0.02, 0.05, null_kp, null_kd, q_ref,
                                                nv, &lambda_sq, nullptr);
    // λ² from the migrated side feeds the oracle: same damping, so the only
    // thing left between them is the factorisation structure.
    const OscInline inline_form =
        PreExtractionOscBlock(s, a_task, lambda_sq, null_kp, null_kd, q_ref, nv);

    ASSERT_TRUE(migrated.dyn_ok) << "tick " << k;
    ASSERT_EQ(migrated.dyn_ok, inline_form.dyn_ok) << "tick " << k;
    worst = std::max(worst, MaxScaledDiff(migrated.tau, inline_form.tau));
  }
  EXPECT_GT(posture_ticks, 0);
  EXPECT_LT(posture_ticks, 60);
  EXPECT_LT(worst, kStructuralTolerance) << "worst scaled difference " << worst;
}

TEST(OscTaskDynamics, RelativeBoundRejectsTheConstantLambdaLaw) {
  // The non-vacuity partner, and the reason tiers 1 and 2 are not one budget:
  // hand the oracle the OLD constant λ² = max(1e-4, 0.05)² while the migrated
  // side runs §6.5, and the SAME comparison must fail. If it did not, 1e-9 would
  // be wide enough to swallow the law change and the test above would be theatre.
  ModelRig rig;
  const int nv = rig.nv();
  compliance::TaskDynamics dyn;
  dyn.Resize(nv, 6);

  std::vector<double> q(static_cast<std::size_t>(nv), 0.0);
  std::vector<double> v(static_cast<std::size_t>(nv), 0.0);
  const Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(nv);
  FillRegularPose(q, v, nv, 3);
  const Snapshot s = rig.At(q, v);
  const Vec6 a_task = Vec6::Constant(0.45);

  double lambda_sq = -1.0;
  const OscInline migrated =
      MigratedOscBlock(dyn, s, a_task, 0.02, 0.05, 3.0, 0.5, q_ref, nv, &lambda_sq, nullptr);
  ASSERT_TRUE(migrated.dyn_ok);
  ASSERT_TRUE(rtc::testing::BitsEqual(lambda_sq, 0.0)) << "pose must be outside the shell";

  constexpr double kOldLambda = 0.05;  // max(1e-4, gains.damping)
  const OscInline constant_lambda =
      PreExtractionOscBlock(s, a_task, kOldLambda * kOldLambda, 3.0, 0.5, q_ref, nv);
  EXPECT_GT(MaxScaledDiff(migrated.tau, constant_lambda.tau), kStructuralTolerance);
}

TEST(ClikDifferentialIk, MatchesThePreExtractionInlineFormWithinRounding) {
  ModelRig rig;
  const int nv = rig.nv();
  compliance::DifferentialIk ik_6d;
  compliance::DifferentialIk ik_3d;
  ik_6d.Resize(nv, 6);
  ik_3d.Resize(nv, 3);

  std::vector<double> q(static_cast<std::size_t>(nv), 0.0);
  std::vector<double> v(static_cast<std::size_t>(nv), 0.0);

  double worst_6d = 0.0;
  double worst_3d = 0.0;
  for (int k = 0; k < 60; ++k) {
    FillRegularPose(q, v, nv, k);
    const Snapshot s = rig.At(q, v);

    Vec6 twist6;
    for (int i = 0; i < 6; ++i)
      twist6[i] = 0.12 * (1.0 + 0.19 * i) - 0.003 * k;
    const Eigen::Vector3d twist3 = twist6.head<3>();
    const Eigen::VectorXd zero_null = Eigen::VectorXd::Zero(nv);

    // ── 6-DOF branch ────────────────────────────────────────────────────────
    const compliance::DifferentialIk::Result r6 = ik_6d.Compute(s.J_full, 0.02, 0.05);
    ASSERT_TRUE(r6.ok) << "tick " << k;
    Eigen::VectorXd dq_migrated = Eigen::VectorXd::Zero(nv);
    ik_6d.Solve(twist6, zero_null, dq_migrated);
    const Eigen::VectorXd dq_inline =
        PreExtractionClikPseudoInverse<6>(s.J_full, r6.lambda_sq) * twist6;
    worst_6d = std::max(worst_6d, MaxScaledDiff(dq_migrated, dq_inline));

    // ── 3-DOF branch ────────────────────────────────────────────────────────
    const compliance::DifferentialIk::Result r3 = ik_3d.Compute(s.J_pos, 0.02, 0.05);
    ASSERT_TRUE(r3.ok) << "tick " << k;
    Eigen::VectorXd dq3_migrated = Eigen::VectorXd::Zero(nv);
    ik_3d.Solve(twist3, zero_null, dq3_migrated);
    const Eigen::VectorXd dq3_inline =
        PreExtractionClikPseudoInverse<3>(s.J_pos, r3.lambda_sq) * twist3;
    worst_3d = std::max(worst_3d, MaxScaledDiff(dq3_migrated, dq3_inline));
  }
  EXPECT_LT(worst_6d, kStructuralTolerance) << "worst 6-DOF scaled difference " << worst_6d;
  EXPECT_LT(worst_3d, kStructuralTolerance) << "worst 3-DOF scaled difference " << worst_3d;
}

TEST(ClikDifferentialIk, RelativeBoundRejectsTheConstantLambdaLaw) {
  ModelRig rig;
  const int nv = rig.nv();
  compliance::DifferentialIk ik_3d;
  ik_3d.Resize(nv, 3);

  std::vector<double> q(static_cast<std::size_t>(nv), 0.0);
  std::vector<double> v(static_cast<std::size_t>(nv), 0.0);
  FillRegularPose(q, v, nv, 5);
  const Snapshot s = rig.At(q, v);
  const Eigen::Vector3d twist(0.21, -0.14, 0.08);

  const compliance::DifferentialIk::Result r = ik_3d.Compute(s.J_pos, 0.02, 0.05);
  ASSERT_TRUE(r.ok);
  ASSERT_TRUE(rtc::testing::BitsEqual(r.lambda_sq, 0.0)) << "pose must be outside the shell";

  Eigen::VectorXd dq_migrated = Eigen::VectorXd::Zero(nv);
  ik_3d.Solve(twist, Eigen::VectorXd::Zero(nv), dq_migrated);

  constexpr double kOldLambda = 0.05;
  const Eigen::VectorXd dq_constant =
      PreExtractionClikPseudoInverse<3>(s.J_pos, kOldLambda * kOldLambda) * twist;
  EXPECT_GT(MaxScaledDiff(dq_migrated, dq_constant), kStructuralTolerance);
}

TEST(ClikNullspace, GainBeforeProjectionMatchesGainAfterWithinRounding) {
  // The fourth structural site, and CLIK's alone: the pre-extraction form
  // projected the raw posture error and scaled AFTER (`null_dq = N·Δq; *= kp`),
  // where routing the term through DifferentialIk::Solve scales BEFORE
  // (`N·(kp·Δq)`). #236 S6 measured 68.36% of doubles differing between the two
  // and deferred the choice here rather than pre-empt this slice. Scaling first
  // is what makes the term joint::ComputePostureVelocity — the same law the
  // other four consumers call — so the duplicate goes away and this is the
  // price. Both sides take N from the SAME Compute(), so the ordering is the
  // only thing under test.
  ModelRig rig;
  const int nv = rig.nv();
  compliance::DifferentialIk ik;
  ik.Resize(nv, 3);

  std::vector<double> q(static_cast<std::size_t>(nv), 0.0);
  std::vector<double> v(static_cast<std::size_t>(nv), 0.0);

  Eigen::VectorXd delta_q = Eigen::VectorXd::Zero(nv);
  for (int i = 0; i < nv; ++i)
    delta_q[i] = 0.23 * (1.0 + 0.31 * i) - 0.4;

  double worst = 0.0;
  for (int k = 0; k < 40; ++k) {
    FillRegularPose(q, v, nv, k);
    const compliance::DifferentialIk::Result r = ik.Compute(rig.At(q, v).J_pos, 0.02, 0.05);
    ASSERT_TRUE(r.ok) << "tick " << k;
    const double kp = 0.5 + 0.05 * k;

    const Eigen::VectorXd scaled_after = kp * (ik.NullspaceProjector() * delta_q);
    const Eigen::VectorXd scaled_before = ik.NullspaceProjector() * (kp * delta_q);
    worst = std::max(worst, MaxScaledDiff(scaled_after, scaled_before));
  }
  EXPECT_LT(worst, kStructuralTolerance) << "worst scaled difference " << worst;
}

// ═══════════════════════════════════════════════════════════════════════════
// Gates and degradation
// ═══════════════════════════════════════════════════════════════════════════

TEST(MigratedGates, OscFallsBackToGravityHoldWhenTheFactorisationFails) {
  // dyn_ok is the branch that decides between the control law and τ = h. It is a
  // bool, so it has no rounding and stays a BIT comparison even in this slice —
  // and with the numerical lanes blunted to a tolerance, the boolean lanes are
  // the sharpest wiring sensor left (S5 M4 / S6 M10).
  ModelRig rig;
  const int nv = rig.nv();
  compliance::TaskDynamics dyn;
  dyn.Resize(nv, 6);

  std::vector<double> q(static_cast<std::size_t>(nv), 0.0);
  std::vector<double> v(static_cast<std::size_t>(nv), 0.0);
  FillRegularPose(q, v, nv, 0);
  Snapshot s = rig.At(q, v);

  const Vec6 a_task = Vec6::Constant(0.2);
  const Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(nv);

  const OscInline good =
      MigratedOscBlock(dyn, s, a_task, 0.02, 0.05, 0.0, 0.0, q_ref, nv, nullptr, nullptr);
  EXPECT_TRUE(good.dyn_ok);
  EXPECT_GT((good.tau - s.h).cwiseAbs().maxCoeff(), 1e-6)
      << "the healthy path must differ from the fallback, or the check below is vacuous";

  // A non-SPD M is the reachable failure: Eigen's LLT rejects a negative pivot.
  // (It does NOT reject NaN — every pivot test is a comparison — which is why a
  // NaN state is not the way to reach this branch; #236 S6 measured that trap.)
  s.M(0, 0) = -1.0;
  const OscInline degraded =
      MigratedOscBlock(dyn, s, a_task, 0.02, 0.05, 0.0, 0.0, q_ref, nv, nullptr, nullptr);
  EXPECT_FALSE(degraded.dyn_ok);
  for (int i = 0; i < nv; ++i)
    EXPECT_TRUE(rtc::testing::BitsEqual(degraded.tau[i], s.h[i])) << "channel " << i;
}

TEST(MigratedGates, ClikHoldsRatherThanCommandingAStaleInverse) {
  // ClikController never checked its LDLT's .info() at all. DifferentialIk keeps
  // the LAST GOOD J⁺/N when it fails, which for a controller that INTEGRATES its
  // output would latch a stale velocity forever — so the adapter zeroes instead.
  // The reachable failure is a non-finite J (a NaN joint state through FK).
  ModelRig rig;
  const int nv = rig.nv();
  compliance::DifferentialIk ik;
  ik.Resize(nv, 3);

  std::vector<double> q(static_cast<std::size_t>(nv), 0.0);
  std::vector<double> v(static_cast<std::size_t>(nv), 0.0);
  FillRegularPose(q, v, nv, 0);
  Eigen::MatrixXd J = rig.At(q, v).J_pos;

  EXPECT_TRUE(ik.Compute(J, 0.02, 0.05).ok);
  J(1, 2) = std::numeric_limits<double>::quiet_NaN();
  const compliance::DifferentialIk::Result bad = ik.Compute(J, 0.02, 0.05);
  EXPECT_FALSE(bad.ok);
  // The stale inverse survives the call — which is exactly why `ok` has to be
  // acted on rather than ignored.
  EXPECT_TRUE(ik.PseudoInverse().allFinite());
}

// ═══════════════════════════════════════════════════════════════════════════
// Adapter-level gates — retired with their subject (#298 S7c-2)
// ═══════════════════════════════════════════════════════════════════════════
//
// `OscAdapter.PostureGateIsWiredToTheGains` and
// `OscAdapter.PostureGateReportsClosedOnAnEstoppedTick` stood here. Both existed
// because a mutation went UNDETECTED without them: #236 R6 budgeted output lanes
// plus shim counters first and authorised a new adapter accessor only where the
// sweep showed a hole.
//
// They are 분류 B — the contract is the GATE's, not the adapter's, so it moved
// rather than retired. Successors: `OscShimPostureGate.IsWiredToTheGains` and
// `OscShimPostureGate.ReportsClosedOnAnEstoppedTick` in test_task_accel_core.cpp,
// driving OscShim with no adapter (S7c-2 2단계, `567678a0`). The E-STOP branch
// they need was added to the shim there; both were mutation-checked to fire on
// their own target and nothing else.

// ── Retired with the adapters (#298 S7c-2 4단계) ────────────────────────────
//
// `ClikAdapter.HoldsInsteadOfCommandingNaNWhenTheJacobianGoesNonFinite` and
// `OscAdapter.HoldsGravityTorqueWhenTheJacobianGoesNonFinite` stood here. Both
// asked the same question of a live adapter: does a non-finite model quantity
// reach the actuator as a command?
//
// They are NOT simply duplicates of the two `MigratedGates` cases above, which
// is what the S7c plan first recorded. Reading them settled it:
//
//   - the core gates pin `!ok` / `!dyn_ok` and that the helper keeps its last
//     good J⁺ / Λ_S. They do not pin what the CALLER does with that answer.
//   - the OSC case's (b) half — a NaN *velocity* — is invisible to any core
//     gate by construction: J(q) and M(q) depend on q alone and stay finite, so
//     `dyn_ok` is true and the τ = h fallback never runs, while h = C(q,v)v
//     carries the NaN through. Only the adapter's Step 10 scrub stopped that
//     tick.
//
// The contract still has an owner after the adapters go, but a different one:
// the base output gate, `RTControllerInterface::ValidateOutput` →
// `OutputRejectReason::kNonFiniteCommand`, covered by
// `rtc_controller_interface/test/test_output_validation.cpp` and
// `rtc_controller_manager/test/test_rt_loop_pipeline.cpp` (both survive S7c).
//
// The ANSWER differs and that is worth writing down: the adapter scrubbed and
// published a usable command, whereas an unscrubbed binding has its whole output
// rejected and the manager substitutes a hold. Both are safe; they are not the
// same behaviour.
//
// ⇒ Binding requirement, not an adapter defect: a future binding that wants to
//   guarantee "my output is finite" — rather than delegating that to the CM's
//   reject-and-hold path — has to reimplement the scrub. Nothing in the core
//   layer will do it for it, because the core cannot see the NaN-velocity case.

TEST(TaskDynamicsCore, ReportsNotOkOnANonFiniteJacobian) {
  // The unit-level statement of the same thing, and the one that pins WHICH
  // signal is load-bearing: without the allFinite() gate this call returns
  // ok == true with sigma_min == 0.0 — a value indistinguishable from a
  // genuinely singular pose, which is precisely why a σ_min fault threshold
  // cannot substitute for the gate.
  ModelRig rig;
  const int nv = rig.nv();
  compliance::TaskDynamics dyn;
  dyn.Resize(nv, 6);

  std::vector<double> q(static_cast<std::size_t>(nv), 0.0);
  std::vector<double> v(static_cast<std::size_t>(nv), 0.0);
  FillRegularPose(q, v, nv, 3);
  const Snapshot s = rig.At(q, v);

  Eigen::LLT<Eigen::MatrixXd> llt_M(nv);
  llt_M.compute(s.M);
  ASSERT_EQ(llt_M.info(), Eigen::Success);

  const compliance::TaskDynamics::Result good = dyn.Compute(s.J_full, llt_M, 0.02, 0.05);
  ASSERT_TRUE(good.ok);
  const Eigen::MatrixXd lambda_good = dyn.LambdaS();

  Eigen::MatrixXd J_nan = s.J_full;
  J_nan(3, 1) = std::numeric_limits<double>::quiet_NaN();
  const compliance::TaskDynamics::Result bad = dyn.Compute(J_nan, llt_M, 0.02, 0.05);
  EXPECT_FALSE(bad.ok) << "a non-finite Jacobian must not report a usable factorisation";
  EXPECT_TRUE(dyn.LambdaS().allFinite()) << "Λ_S must keep its last good contents, not become NaN";
  EXPECT_TRUE(dyn.NullspaceProjectorTranspose().allFinite());
  for (Eigen::Index i = 0; i < lambda_good.size(); ++i) {
    EXPECT_TRUE(rtc::testing::BitsEqual(dyn.LambdaS()(i), lambda_good(i)))
        << "a rejected call must leave Λ_S untouched, not partially written; element " << i;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// RT-1 — the migrated tick still allocates nothing
// ═══════════════════════════════════════════════════════════════════════════

TEST(MigratedRtSafety, IsAllocationFree) {
  // Both gates, because they see different things: ScopedAllocGate counts
  // operator new, ScopedNoMalloc rides eigen_assert and catches Eigen's
  // aligned_malloc — which goes straight to std::malloc and is invisible to the
  // first. The specific risk this migration introduces is an Eigen::Ref argument
  // whose layout does not match, which copies to a heap buffer silently.
  ModelRig rig;
  const int nv = rig.nv();
  compliance::TaskDynamics dyn;
  compliance::DifferentialIk ik_6d;
  compliance::DifferentialIk ik_3d;
  dyn.Resize(nv, 6);
  ik_6d.Resize(nv, 6);
  ik_3d.Resize(nv, 3);

  std::vector<double> q(static_cast<std::size_t>(nv), 0.0);
  std::vector<double> v(static_cast<std::size_t>(nv), 0.0);
  FillRegularPose(q, v, nv, 7);
  const Snapshot s = rig.At(q, v);

  Eigen::LLT<Eigen::MatrixXd> llt_M(nv);
  llt_M.compute(s.M);
  ASSERT_EQ(llt_M.info(), Eigen::Success);

  const Vec6 a_task = Vec6::Constant(0.3);
  const Vec6 twist6 = Vec6::Constant(0.1);
  const Eigen::Vector3d twist3(0.1, 0.1, 0.1);
  Eigen::VectorXd tau0 = Eigen::VectorXd::Zero(nv);
  Eigen::VectorXd null_tmp = Eigen::VectorXd::Zero(nv);
  Eigen::VectorXd qdot_null = Eigen::VectorXd::Zero(nv);
  Eigen::VectorXd dq = Eigen::VectorXd::Zero(nv);
  Eigen::VectorXd traj_dq = Eigen::VectorXd::Zero(nv);
  Vec6 F = Vec6::Zero();

  // Warm up outside the gates: the first call through a decomposition may size
  // internals, and Resize() is the off-RT step that is allowed to allocate.
  (void)dyn.Compute(s.J_full, llt_M, 0.02, 0.05);
  (void)ik_6d.Compute(s.J_full, 0.02, 0.05);
  (void)ik_3d.Compute(s.J_pos, 0.02, 0.05);
  ik_6d.Solve(twist6, qdot_null, dq);
  ik_3d.Solve(twist3, qdot_null, dq);

  {
    const rtc::testing::ScopedAllocGate new_gate;
    const rtc::testing::ScopedNoMalloc eigen_gate;

    // OSC's migrated Steps 7-9.
    const compliance::TaskDynamics::Result r = dyn.Compute(s.J_full, llt_M, 0.02, 0.05);
    F.noalias() = dyn.LambdaS() * a_task;
    dyn.ProjectNullspace(tau0, null_tmp);

    // CLIK's migrated Steps 4-6, both branches.
    const compliance::DifferentialIk::Result r6 = ik_6d.Compute(s.J_full, 0.02, 0.05);
    ik_6d.Solve(twist6, dq);  // 6-DOF mode: the no-posture overload, as the adapter calls it
    ik_6d.Solve(twist6, qdot_null, dq);
    traj_dq.noalias() = ik_6d.PseudoInverse() * twist6;

    const compliance::DifferentialIk::Result r3 = ik_3d.Compute(s.J_pos, 0.02, 0.05);
    ik_3d.Solve(twist3, qdot_null, dq);
    traj_dq.noalias() = ik_3d.PseudoInverse() * twist3;

    EXPECT_TRUE(r.ok);
    EXPECT_TRUE(r6.ok);
    EXPECT_TRUE(r3.ok);
  }
  EXPECT_TRUE(F.allFinite());
  EXPECT_TRUE(dq.allFinite());
  EXPECT_TRUE(traj_dq.allFinite());
}

}  // namespace
