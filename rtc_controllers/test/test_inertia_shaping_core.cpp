// ── Golden-vector harness for the §6.3 inertia-shaping core (#236 S4) ────────
// Fourth slice of the rtc_controllers pure-library refactor, on the contract S1
// established and S2a/S3a followed (test_joint_pd_core.cpp,
// test_task_accel_core.cpp, test_task_vel_core.cpp): the claim under test is not
// "the new core is correct" but the stronger "the extraction changed NOTHING",
// so every comparison is bitwise (rtc::testing::BitsEqual), never a tolerance.
//
// Two oracles, deliberately:
//
//   1. PreExtractionInlineForm — a literal, rename-only copy of
//      TaskImpedanceController::ApplyInertiaShaping (task_impedance_controller.
//      cpp:247-327 at 697f16d2), keeping the Eigen::MatrixXd scratch the member
//      version used. DURABLE: it outlives the adapter, so it still pins the core
//      after S7 deletes TaskImpedanceController — and it is what MEASURES the one
//      thing S4 changed on purpose, the scratch type. D-S4a moved Λ_d, Bᵀ and
//      their Cholesky from MatrixXd members onto a max-size
//      Matrix<double,Dyn,Dyn,0,6,6> local so the core could be stateless and
//      still heap-free. An independent probe said that was bit-inert for m = 3..6
//      before the code was written; this oracle re-measures it in-tree, on every
//      run, rather than inheriting the claim.
//
//   2. The live TaskImpedanceController. TRANSIENT (dies with S7). Its job is the
//      one thing oracle 1 structurally cannot do: catch a CORRELATED
//      transcription error, plus the WIRING — which Λ reaches the core, whether
//      f_ext arrives ramped by α, whether the two diagnostic flags still land on
//      their own fields, and whether the shaping is skipped exactly when the
//      adapter skips it.
//
// The observability here is as bad as the OSC's (plan §S4 R5): there is no
// f_cmd accessor, and GetDiagnosticsForTesting() exposes only the two flags. So
// ImpedanceShim replicates the whole τ tail — Λ_S/Nᵀ, JᵀSᵀ, the posture task,
// ĝ(q) — and the comparison is on the final per-joint torque. Everything it
// replicates is something S7's binding will have to own, which is what makes the
// shim a prototype of that binding rather than a second implementation.
//
// The ONE lane the shim borrows instead of replicating is the conditioned
// external wrench: it reads diag.wrench_lwa (the value UpdateExternalWrench
// returned this tick) and applies α itself. The wrench pipeline is upstream of
// the law and out of S4's scope; what stays pinned is the α scaling and the
// (B − I)·f_ext wiring, which is what this slice is about.
//
// The shim reads its model from a SECOND, independently built RtModelHandle.
// That premise is measured, not assumed:
// ReferenceDynamics.IndependentHandlesAgreeBitwise (test_task_accel_core.cpp)
// pins the dynamics half — J, M, h, placement — on this same serial_7dof
// fixture, and ReferenceHandles.IndependentHandlesAgreeOnTheReorderPathBitwise
// (test_task_vel_core.cpp) pins the reorder half, which this shim also uses
// (posture gather, torque scatter). Per plan §S4 R5 this slice REUSES both
// measurements rather than repeating them; that reuse is valid only while all
// three files share serial7dof_fixture.hpp, which they do.
//
// Every bitwise suite carries a non-vacuity partner
// (BitwiseComparisonRejectsAMaterialisedInverse) proving the comparison can
// actually reject an algebraically equivalent regrouping — and it is the
// regrouping the probe measured as sensitive, not merely a different spelling of
// the same triangular solve.

// no_malloc_scope.hpp MUST precede every Eigen header — it installs the Eigen
// allocation tripwire by defining EIGEN_RUNTIME_NO_MALLOC and its own
// eigen_assert, and a later include would be silently ignored (its own #error
// enforces the ordering). Everything below pulls Eigen, so it comes first.
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_controllers/compliance/impedance_law.hpp"
#include "rtc_controllers/compliance/inertia_shaping.hpp"
#include "rtc_controllers/compliance/task_dynamics.hpp"
#include "rtc_controllers/direct/task_impedance_controller.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/bit_compare.hpp"
#include "rtc_controllers/testing/serial7dof_fixture.hpp"
#include "rtc_math/se3/pinocchio_adapter.hpp"
#include "rtc_math/se3/pose_error.hpp"
#include "test_urdf_path.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <random>
#include <span>
#include <string>
#include <vector>

// The allocation gate is TWO complementary sensors, and this core needs both:
//   • rtc::testing::ScopedAllocGate (rtc_controllers/testing/alloc_gate.hpp)
//     counts `operator new`, so it sees a std::vector or a header-inline helper.
//   • rtc::testing::ScopedNoMalloc (rtc_base/testing/no_malloc_scope.hpp) rides
//     eigen_assert, so it sees EIGEN allocation — which the first gate CANNOT:
//     Eigen's internal::aligned_malloc calls std::malloc directly and never
//     routes through operator new. This core's most plausible RT-1 regression is
//     exactly that: the max-size scratch decaying into a plain MatrixXd, which
//     compiles, runs, produces identical numbers, and allocates on every tick.
// IsAllocationFree arms both.

namespace {

using Vec6 = Eigen::Matrix<double, 6, 1>;

using rtc::kMaxDeviceChannels;
using rtc::TaskImpedanceController;
using rtc::compliance::ComputeShapedTaskForce;
using rtc::compliance::InertiaShapingParams;
using rtc::compliance::InertiaShapingResult;
using rtc::testing::BitsEqual;
using rtc::testing::FillSweep;
using rtc::testing::MakeHandle;
using rtc::testing::MakeRng;
using rtc::testing::MakeState;
using rtc::testing::Serial7dof;

using Sel = TaskImpedanceController::TaskSelection;
using Form = TaskImpedanceController::Formulation;

constexpr double kDt = 0.002;

// task_impedance_controller.cpp's file-local NUM-1 floor, applied at the point
// of use by PR #270 (#257) — the shim has to reproduce the FLOORED argument,
// because that floored form is what the golden vector fixes (plan §S4 R7).
constexpr double kMinMaxDamping = 1e-4;

// ═══════════════════════════════════════════════════════════════════════════
// Oracle 1 — the literal pre-extraction form
// ═══════════════════════════════════════════════════════════════════════════

// One tick's worth of core inputs. Λ_S arrives as a plain MatrixXd because that
// is what TaskDynamics::LambdaS() hands over and what the pre-extraction member
// consumed.
struct CoreDraw {
  int m{6};
  Eigen::MatrixXd lambda_s;
  Vec6 f_task{Vec6::Zero()};
  Vec6 f_ext{Vec6::Zero()};
  std::array<double, 6> desired_inertia{{1.0, 1.0, 1.0, 0.1, 0.1, 0.1}};
  bool natural{true};
  double max_inertia_ratio{3.0};
};

// The three outputs the pre-extraction code produced: f_cmd (written through a
// reference) and the two Diagnostics flags.
struct ShapingOutcome {
  Vec6 f_cmd{Vec6::Zero()};
  bool solve_failed{false};
  bool clamped{false};
};

// EXACTLY the expression as it stood inline in
// TaskImpedanceController::ApplyInertiaShaping before #236 S4 lifted it into
// compliance/inertia_shaping.hpp (task_impedance_controller.cpp:247-327 at
// 697f16d2). Rename-only transcription:
//   gains.desired_inertia_natural → d.natural   dyn_.LambdaS()   → d.lambda_s
//   gains.desired_inertia         → d.desired_inertia
//   gains.max_inertia_ratio       → d.max_inertia_ratio
//   m_                            → d.m
//   diag.inertia_solve_failed     → out.solve_failed
//   diag.inertia_clamped          → out.clamped
// Nothing else moved: same operation order, same association, same expression
// shapes — and crucially the SAME Eigen types, Eigen::MatrixXd scratch with an
// LLT<MatrixXd>, which is what makes this oracle a measurement of D-S4a's
// max-size scratch rather than a restatement of it.
ShapingOutcome PreExtractionInlineForm(const CoreDraw& d) {
  ShapingOutcome out;
  out.f_cmd = d.f_task;  // the adapter initialised f_cmd = f_task before the call

  Eigen::MatrixXd lambda_d_ = Eigen::MatrixXd::Identity(d.m, d.m);
  Eigen::MatrixXd b_transp_ = Eigen::MatrixXd::Identity(d.m, d.m);
  Eigen::LLT<Eigen::MatrixXd> llt_lambda_d_(d.m);
  const Eigen::MatrixXd& lambda_s = d.lambda_s;

  if (d.natural) {
    lambda_d_ = lambda_s;
  } else {
    lambda_d_.setZero();
    for (int i = 0; i < d.m; ++i)
      lambda_d_(i, i) = d.desired_inertia[static_cast<std::size_t>(i)];
  }

  llt_lambda_d_.compute(lambda_d_);
  if (llt_lambda_d_.info() != Eigen::Success) {
    out.solve_failed = true;
    return out;
  }

  b_transp_ = lambda_s;
  llt_lambda_d_.solveInPlace(b_transp_);

  double dev_inf = 0.0;
  for (int i = 0; i < d.m; ++i) {
    double row = 0.0;
    for (int j = 0; j < d.m; ++j)
      row += std::abs(b_transp_(j, i) - (i == j ? 1.0 : 0.0));
    dev_inf = std::max(dev_inf, row);
  }
  const double ratio_max = std::max(1.0, d.max_inertia_ratio);
  double s = 1.0;
  if (dev_inf > ratio_max - 1.0) {
    s = (ratio_max - 1.0) / dev_inf;
    out.clamped = true;
  }

  for (int i = 0; i < d.m; ++i) {
    double acc = 0.0;
    for (int j = 0; j < d.m; ++j) {
      const double eye = (i == j) ? 1.0 : 0.0;
      const double b_ij = eye + s * (b_transp_(j, i) - eye);
      acc += b_ij * d.f_task(j) + (b_ij - eye) * d.f_ext(j);
    }
    out.f_cmd(i) = acc;
  }
  return out;
}

// Drive the core with the same draw — this is the marshalling the binding does.
ShapingOutcome RunCore(const CoreDraw& d) {
  const InertiaShapingResult r = ComputeShapedTaskForce(
      InertiaShapingParams{d.desired_inertia, d.natural, d.max_inertia_ratio}, d.lambda_s, d.f_task,
      d.f_ext, d.m);
  return ShapingOutcome{r.f_cmd, r.solve_failed, r.clamped};
}

// A symmetric positive-definite Λ_S of the magnitude a real task inertia has
// (O(0.1)–O(10) kg on the linear axes, smaller on the angular ones), built as
// L·Lᵀ + εI so the Cholesky the law runs actually succeeds.
Eigen::MatrixXd RandomSpd(std::mt19937& rng, int m) {
  std::normal_distribution<double> draw(0.0, 1.0);
  Eigen::MatrixXd L = Eigen::MatrixXd::Zero(m, m);
  for (int i = 0; i < m; ++i)
    for (int j = 0; j <= i; ++j)
      L(i, j) = draw(rng);
  Eigen::MatrixXd A = L * L.transpose();
  A += 0.05 * Eigen::MatrixXd::Identity(m, m);
  return A;
}

// Draws sweep the regimes the law actually meets, and every branch it has:
// natural (B = I through the real solve) and shaped, clamp inert and clamp
// engaged, m = 6 and m = 3, plus a Λ_d that cannot be factorised at all. The
// branch counters below turn "these were exercised" from a claim into an
// assertion — the S2a lesson: a test that names a branch and never fires it
// pins nothing.
struct BranchCounts {
  int natural{0};
  int shaped{0};
  int clamped{0};
  int solve_failed{0};
  int m3{0};
  int m6{0};
};

CoreDraw RandomDraw(std::mt19937& rng, int trial) {
  std::normal_distribution<double> force(0.0, 30.0);
  std::uniform_real_distribution<double> inertia(0.05, 8.0);

  CoreDraw d;
  d.m = (trial % 3 == 0) ? 3 : 6;  // TRANSLATION_ONLY and FULL_SE3, the only two
  d.lambda_s = RandomSpd(rng, d.m);
  d.natural = (trial % 2 == 0);
  for (std::size_t i = 0; i < 6; ++i)
    d.desired_inertia[i] = inertia(rng);
  for (int i = 0; i < 6; ++i) {
    d.f_task(i) = force(rng);
    d.f_ext(i) = force(rng);
  }
  // Every fourth trial takes the ratio bound down to where the clamp has to
  // engage; every eleventh puts it far out of reach so `s` stays exactly 1.
  d.max_inertia_ratio = (trial % 4 == 0) ? 1.05 : ((trial % 11 == 0) ? 1.0e9 : 3.0);
  // Every seventh draw is NOT positive definite, so the Cholesky fails and the
  // degrade-to-B=I path runs. Negating a diagonal entry is enough and keeps the
  // matrix symmetric, which is the shape a broken Λ_S actually has.
  if (trial % 7 == 0 && d.m > 1) {
    d.natural = true;  // Λ_d := Λ_S ⇒ the bad matrix is the one factorised
    d.lambda_s(1, 1) = -std::abs(d.lambda_s(1, 1)) - 1.0;
  }
  // A term is zeroed every so often so no axis is permanently masked by a larger
  // one — a cancellation that only shows up when the terms are comparable is
  // precisely what a regrouping changes.
  if (trial % 5 == 0)
    d.f_ext.setZero();
  if (trial % 13 == 0)
    d.f_task.setZero();
  return d;
}

void Tally(const CoreDraw& d, const ShapingOutcome& o, BranchCounts* c) {
  (d.natural ? c->natural : c->shaped)++;
  (d.m == 3 ? c->m3 : c->m6)++;
  if (o.clamped)
    c->clamped++;
  if (o.solve_failed)
    c->solve_failed++;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Bit-identity against the literal pre-extraction form
// ═══════════════════════════════════════════════════════════════════════════

TEST(InertiaShaping, MatchesThePreExtractionInlineFormBitwise) {
  auto rng = MakeRng();
  double peak = 0.0;
  BranchCounts counts;

  for (int trial = 0; trial < 600; ++trial) {
    const CoreDraw d = RandomDraw(rng, trial);

    const ShapingOutcome got = RunCore(d);
    const ShapingOutcome want = PreExtractionInlineForm(d);
    Tally(d, want, &counts);

    EXPECT_EQ(got.solve_failed, want.solve_failed) << "trial " << trial << ": solve_failed";
    EXPECT_EQ(got.clamped, want.clamped) << "trial " << trial << ": clamped";
    for (int i = 0; i < 6; ++i) {
      EXPECT_TRUE(BitsEqual(got.f_cmd(i), want.f_cmd(i)))
          << "trial " << trial << " row " << i << ": " << got.f_cmd(i) << " vs " << want.f_cmd(i);
      peak = std::max(peak, std::abs(got.f_cmd(i)));
    }
  }

  EXPECT_GT(peak, 1e-6) << "every compared row was ~0 — the draw generator degenerated";
  // The branches the law HAS, each measured to have run. Without these the suite
  // could be pinning one path and calling it the law (plan §S4 R6).
  EXPECT_GT(counts.natural, 0) << "the natural (Λ_d := Λ_S ⇒ B = I) path never ran";
  EXPECT_GT(counts.shaped, 0) << "the shaped (diagonal Λ_d) path never ran";
  EXPECT_GT(counts.clamped, 0) << "the §5.2 deviation clamp never engaged";
  EXPECT_GT(counts.solve_failed, 0) << "the non-factorable Λ_d degrade path never ran";
  EXPECT_GT(counts.m3, 0) << "TRANSLATION_ONLY (m = 3) never ran — the max-size scratch is the "
                             "one thing S4 changed, and m < 6 is where it matters";
  EXPECT_GT(counts.m6, 0) << "FULL_SE3 (m = 6) never ran";
}

// Non-vacuity partner, and NOT a different spelling of the same algorithm: the
// probe that chose D-S4a found that an out-of-place triangular solve is bitwise
// identical to solveInPlace, so a mutant built that way would prove nothing.
// This one materialises Λ_d⁻¹ and multiplies — a genuinely different route to
// the same B, which the probe measured as differing on ~half of 720k doubles.
// If bit equality could not tell these apart, the test above would be pinning
// nothing.
TEST(InertiaShaping, BitwiseComparisonRejectsAMaterialisedInverse) {
  auto rng = MakeRng();
  int differing = 0;
  int compared = 0;

  for (int trial = 0; trial < 600; ++trial) {
    const CoreDraw d = RandomDraw(rng, trial);
    const ShapingOutcome got = RunCore(d);
    if (got.solve_failed)
      continue;  // no B was formed; nothing to regroup

    // Same B, different route: invert Λ_d, then multiply.
    Eigen::MatrixXd lambda_d = Eigen::MatrixXd::Zero(d.m, d.m);
    if (d.natural) {
      lambda_d = d.lambda_s;
    } else {
      for (int i = 0; i < d.m; ++i)
        lambda_d(i, i) = d.desired_inertia[static_cast<std::size_t>(i)];
    }
    Eigen::LLT<Eigen::MatrixXd> llt(lambda_d);
    if (llt.info() != Eigen::Success)
      continue;
    const Eigen::MatrixXd lambda_d_inv = llt.solve(Eigen::MatrixXd::Identity(d.m, d.m));
    const Eigen::MatrixXd b_mat = d.lambda_s * lambda_d_inv;  // B, not Bᵀ

    double dev_inf = 0.0;
    for (int i = 0; i < d.m; ++i) {
      double row = 0.0;
      for (int j = 0; j < d.m; ++j)
        row += std::abs(b_mat(i, j) - (i == j ? 1.0 : 0.0));
      dev_inf = std::max(dev_inf, row);
    }
    const double ratio_max = std::max(1.0, d.max_inertia_ratio);
    const double s = (dev_inf > ratio_max - 1.0) ? (ratio_max - 1.0) / dev_inf : 1.0;

    for (int i = 0; i < d.m; ++i) {
      double acc = 0.0;
      for (int j = 0; j < d.m; ++j) {
        const double eye = (i == j) ? 1.0 : 0.0;
        const double b_ij = eye + s * (b_mat(i, j) - eye);
        acc += b_ij * d.f_task(j) + (b_ij - eye) * d.f_ext(j);
      }
      ++compared;
      if (!BitsEqual(got.f_cmd(i), acc))
        ++differing;
      // The mutant must stay algebraically equivalent, else it proves nothing.
      // The tolerance rides the row magnitude because a poorly conditioned Λ_d
      // makes both routes large, not because either is wrong.
      const double scale = 1.0 + std::abs(got.f_cmd(i)) + std::abs(acc);
      EXPECT_NEAR(got.f_cmd(i), acc, 1e-6 * scale) << "trial " << trial << " row " << i;
    }
  }

  ASSERT_GT(compared, 0) << "no rows compared — the draw generator degenerated";
  EXPECT_GT(differing, 0) << "bit comparison cannot distinguish a materialised inverse — the "
                             "equivalence test is vacuous";
}

// The m argument is clamped to [0,6], which bounds the fixed 6-VECTORS and
// nothing else: Λ_S arrives as a Ref and carries its own size. Pairing m = 6
// with a 3×3 Λ_S therefore used to have topLeftCorner(6,6) read 27 doubles past
// the buffer, and eigen_assert cannot be the sensor for it — the library ships
// -DNDEBUG, where that assert is compiled out and the shaped force is built from
// whatever follows Λ_S in memory, with every number finite and no fault raised.
//
// Note the two failure modes if the guard is removed, because they differ by TU:
// here eigen_assert is live (no_malloc_scope.hpp installs its own, which aborts
// outside a ScopedNoMalloc region), so this test CRASHES rather than fails; in
// the shipped library the same defect is silent. That asymmetry is the reason
// the guard is a runtime check and not an assertion.
TEST(InertiaShaping, UndersizedLambdaSDegradesInsteadOfReadingPastIt) {
  auto rng = MakeRng();
  const Eigen::MatrixXd lambda_s3 = RandomSpd(rng, 3);

  Vec6 f_task;
  f_task << 11.0, -7.0, 3.5, 0.9, -0.4, 0.2;
  Vec6 f_ext;
  f_ext << -2.0, 1.5, 0.5, 0.1, 0.05, -0.2;

  const InertiaShapingParams p{{{0.4, 0.4, 0.4, 0.03, 0.03, 0.03}}, /*natural=*/false, 3.0};

  // m = 6 against a 3×3 Λ_S: the caller error. Degrades exactly as a
  // non-factorable Λ_d does, so the caller's existing handling covers it.
  const InertiaShapingResult undersized = ComputeShapedTaskForce(p, lambda_s3, f_task, f_ext, 6);
  EXPECT_TRUE(undersized.solve_failed) << "an undersized Λ_S must degrade, not be shaped";
  EXPECT_FALSE(undersized.clamped) << "the §5.2 clamp cannot have run — the law never reached it";
  for (int i = 0; i < 6; ++i)
    EXPECT_TRUE(BitsEqual(undersized.f_cmd(i), f_task(i)))
        << "row " << i << ": the degrade must pass the §6.2 force through untouched";

  // Non-vacuity: the SAME Λ_S at the m it actually fits must shape normally, or
  // the assertions above would also hold for a law that degraded unconditionally.
  const InertiaShapingResult fitted = ComputeShapedTaskForce(p, lambda_s3, f_task, f_ext, 3);
  EXPECT_FALSE(fitted.solve_failed) << "a Λ_S of exactly m×m is not undersized";
  bool any_shaped = false;
  for (int i = 0; i < 3; ++i)
    any_shaped = any_shaped || !BitsEqual(fitted.f_cmd(i), f_task(i));
  EXPECT_TRUE(any_shaped) << "the fitted draw produced f_task unchanged — the contrast is vacuous";

  // An OVERSIZED Λ_S stays legal: the doc contract is "at least m×m", and the
  // adapter relies on it the moment a task narrower than Λ_S is selected.
  const Eigen::MatrixXd lambda_s6 = RandomSpd(rng, 6);
  const InertiaShapingResult oversized = ComputeShapedTaskForce(p, lambda_s6, f_task, f_ext, 3);
  EXPECT_FALSE(oversized.solve_failed) << "an Λ_S larger than m×m must still be shaped";
}

// ═══════════════════════════════════════════════════════════════════════════
// Cross-check against the live adapter (transient — dies with S7)
// ═══════════════════════════════════════════════════════════════════════════

namespace {

// Owns the model, the dynamics helper and the posture task (STRUCTURE) and calls
// the core (ALGORITHM). Mirrors TaskImpedanceController::Compute() step for step
// from the joint-state copy down to the device-order torque; any divergence here
// is a divergence the future binding would ship.
class ImpedanceShim {
 public:
  explicit ImpedanceShim(int m) : m_(m) {
    handle_ = MakeHandle(model_);
    tip_ = static_cast<pinocchio::FrameIndex>(model_->nframes - 1);
    nv_ = handle_->nv();

    J_full_ = Eigen::MatrixXd::Zero(6, nv_);
    J_S_ = Eigen::MatrixXd::Zero(m_, nv_);
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
    dyn_.Resize(nv_, m_);
  }

  [[nodiscard]] int nv() const { return nv_; }

  // The REGULATED task rows. The cross-check bounds its non-vacuity guards by
  // this rather than by 6: under TRANSLATION_ONLY the rotation rows of `e` are
  // left to the posture task and drift by design, so a guard that reads all six
  // can be satisfied by drift alone while the branch it claims to prove is inert.
  [[nodiscard]] int task_dim() const { return m_; }

  [[nodiscard]] int clamps() const { return clamps_; }

  [[nodiscard]] int solve_failures() const { return solve_failures_; }

  [[nodiscard]] int ramped_ticks() const { return ramped_ticks_; }

  [[nodiscard]] int wrench_ticks() const { return wrench_ticks_; }

  [[nodiscard]] int nullspace_ticks() const { return nullspace_ticks_; }

  [[nodiscard]] double alpha() const { return alpha_; }

  // One tick. `f_lwa` is the conditioned external wrench the adapter produced on
  // THIS tick (diag.wrench_lwa) — the one lane the shim borrows rather than
  // replicates; see the file header. Returns the torque in DEVICE channel order,
  // before the adapter's §10.5 safety layer and output clamp (both asserted
  // inert by the caller, which is what keeps the comparison meaningful).
  Eigen::VectorXd Step(const TaskImpedanceController::Gains& g, bool inertia_shaping,
                       const rtc::ControllerState& state, const std::array<double, 6>& f_lwa) {
    // Mirrors the adapter's own local, and for the same reason: a -Warray-bounds
    // fix, NOT part of the contract this file compares. m_ is fixed at 3 or 6 by
    // construction, so the clamp cannot bite in either place — it exists only so
    // the compiler can see the bound across the inlined helper. Nothing in the
    // bitwise comparison depends on it.
    const int m = std::clamp(m_, 0, 6);
    const auto& dev0 = state.devices[0];
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

    // Seed X_d and the posture target from the measurement, as the adapter's
    // !target_initialized_ branch does (§10.7).
    if (!initialized_) {
      goal_pose_ = tcp;
      for (int i = 0; i < nv_; ++i)
        q_null_(i) = q_dev_(i);
      elapsed_ = 0.0;
      initialized_ = true;
    }

    const Vec6 e =
        rtc::math::se3::computePoseError(tcp, goal_pose_, rtc::math::se3::ErrorType::SplitWorld);
    const Vec6 nu_d = Vec6::Zero();

    const double ramp = g.activation_ramp_time;
    alpha_ = (ramp <= 0.0) ? 1.0 : std::min(1.0, elapsed_ / ramp);
    if (alpha_ > 0.0 && alpha_ < 1.0)
      ++ramped_ticks_;

    J_S_ = J_full_.topRows(m);
    const Vec6 f_task =
        rtc::compliance::ComputeImpedanceForce(g.impedance, e, tcp_vel_, nu_d, alpha_, m);

    handle_->ComputeGeneralizedGravity(q_span);
    gravity_ = handle_->GetGeneralizedGravity();

    Vec6 f_ext_raw;
    for (int i = 0; i < 6; ++i)
      f_ext_raw(i) = f_lwa[static_cast<std::size_t>(i)];
    if (f_ext_raw.norm() > 0.0)
      ++wrench_ticks_;
    const Vec6 f_ext = alpha_ * f_ext_raw;

    bool dyn_ok = true;
    // #277's `FloorPostureGain` sits immediately above this line in the adapter
    // and is deliberately not mirrored: it is a binding-side clamp, and every
    // gain this shim is driven with is non-negative, where the floor is the
    // identity. A negative gain here would make the two sides disagree — floor
    // at the CALL SITE if one is ever added.
    const bool nullspace_active = (nv_ > m) && (g.nullspace_kp != 0.0 || g.nullspace_kd != 0.0);
    const bool need_task_dynamics = nullspace_active || inertia_shaping;
    Vec6 f_cmd = f_task;
    if (need_task_dynamics) {
      handle_->ComputeMassMatrix(q_span);
      M_ = handle_->GetMassMatrix();
      M_.triangularView<Eigen::StrictlyLower>() =
          M_.triangularView<Eigen::StrictlyUpper>().transpose();
      llt_M_.compute(M_);
      if (llt_M_.info() != Eigen::Success) {
        dyn_ok = false;
      } else {
        const rtc::compliance::TaskDynamics::Result r = dyn_.Compute(
            J_S_, llt_M_, g.singularity_threshold, std::max(kMinMaxDamping, g.max_damping));
        dyn_ok = r.ok;
        if (dyn_ok && inertia_shaping) {
          const InertiaShapingResult shaped =
              ComputeShapedTaskForce(g.inertia, dyn_.LambdaS(), f_task, f_ext, m);
          f_cmd = shaped.f_cmd;
          if (shaped.clamped)
            ++clamps_;
          if (shaped.solve_failed)
            ++solve_failures_;
        }
        if (dyn_ok && nullspace_active) {
          for (int i = 0; i < nv_; ++i)
            tau_posture_dev_(i) =
                g.nullspace_kp * (q_null_(i) - q_dev_(i)) - g.nullspace_kd * qdot_dev_(i);
          handle_->ReorderInput(
              std::span<const double>(tau_posture_dev_.data(), static_cast<std::size_t>(nv_)),
              tau_posture_);
          dyn_.ProjectNullspace(tau_posture_, tau_null_);
          ++nullspace_ticks_;
        }
      }
    }

    tau_.noalias() = J_S_.transpose() * f_cmd.head(m);
    if (dyn_ok && nullspace_active)
      tau_.noalias() += alpha_ * tau_null_;
    if (dyn_ok) {
      tau_ += gravity_;
    } else {
      tau_ = gravity_;
    }

    handle_->ReorderOutput(tau_, std::span<double>(tau_dev_.data(), static_cast<std::size_t>(nv_)));
    elapsed_ += state.dt;  // the adapter advances the ramp on a clean control tick
    return tau_dev_;
  }

 private:
  int m_;
  int nv_{0};
  std::shared_ptr<const pinocchio::Model> model_;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle_;
  pinocchio::FrameIndex tip_{0};
  rtc::compliance::TaskDynamics dyn_;

  bool initialized_{false};
  double elapsed_{0.0};
  double alpha_{0.0};
  int clamps_{0};
  int solve_failures_{0};
  int ramped_ticks_{0};
  int wrench_ticks_{0};
  int nullspace_ticks_{0};

  pinocchio::SE3 goal_pose_{pinocchio::SE3::Identity()};
  Eigen::MatrixXd J_full_;
  Eigen::MatrixXd J_S_;
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

// A wrench source with no deadband, no filter and no bias calibration: this
// slice is about the §6.3 law, so every upstream stage that could re-shape
// f_ext is switched off rather than replicated (each has its own test in
// test_task_impedance_wrench.cpp).
//
// `formulation` is a PARAMETER because the §6.3 binding is a gated one: the
// suite has to run the gate closed as well as open, or a regression that drops
// `&& inertia_shaping` from the adapter would shape a jacobian_transpose tick
// with nothing to notice.
std::string CrossCheckYaml(const std::string& formulation) {
  return "formulation: " + formulation + R"(
external_wrench:
  enabled: true
  filter_enabled: false
  bias_calibration_samples: 0
  deadband: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  max: [1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6]
)";
}

TaskImpedanceController::Gains CrossCheckGains() {
  TaskImpedanceController::Gains g;
  g.impedance.kp_pos = {{120.0, 120.0, 120.0}};
  g.impedance.kd_pos = {{18.0, 18.0, 18.0}};
  g.impedance.kp_rot = {{12.0, 12.0, 12.0}};
  g.impedance.kd_rot = {{4.0, 4.0, 4.0}};
  g.nullspace_kp = 2.0;  // nv = 7 > m ⇒ Nᵀ is non-trivial on both selections
  g.nullspace_kd = 0.5;
  g.activation_ramp_time = 0.0;  // α = 1 unless a case says otherwise
  // The §10.5 rate limit is a SAFETY-LAYER parameter, not a law gain, and a
  // bitwise comparison is void the moment it fires (the shim stops at the law).
  // A shaped Λ_d legitimately produces a torque step of tens of N·m on the tick
  // the wrench first lands — the default 2000 N·m/s allows 4 N·m per 2 ms tick,
  // so the layer would bite on the very transient this suite exists to compare.
  // Raising it moves the OPERATING POINT (S3a's lesson) instead of shrinking the
  // Λ_d contrast, which would make the shaped cases indistinguishable from the
  // natural ones. That it stays inert is still asserted every tick, not assumed.
  g.max_torque_rate = 1.0e6;
  return g;
}

struct CrossCheckStats {
  double peak_tau{0.0};
  double peak_err{0.0};
  int adapter_clamps{0};
};

// Drive adapter and shim through the same tick sequence and compare every
// device-0 torque bit for bit. `inertia_shaping` is the GATE, not a constant:
// the shim is told to shape exactly when the adapter's formulation says to, so
// the comparison pins the gate itself and not merely the shaped arithmetic.
void RunCrossCheck(TaskImpedanceController& ctrl, ImpedanceShim& shim,
                   const TaskImpedanceController::Gains& g, bool inertia_shaping, int ticks,
                   CrossCheckStats* stats) {
  const int nv = shim.nv();
  auto state = MakeState(nv, kDt);

  for (int tick = 0; tick < ticks; ++tick) {
    FillSweep(state, nv, tick, kDt);

    // A wrench that changes sign and magnitude, so the (B − I)·f_ext term is not
    // a constant offset the comparison could absorb.
    const double phase = 0.05 * tick;
    const std::array<double, 6> w{
        4.0 * std::cos(phase), 3.0 * std::sin(phase), -2.5, 0.4 * std::sin(phase), -0.3, 0.2};
    ctrl.SetExternalWrench(std::span<const double, 6>(w.data(), 6));

    const auto out = ctrl.Compute(state);
    const auto diag = ctrl.GetDiagnosticsForTesting();
    ASSERT_TRUE(diag.control_valid) << "tick " << tick << ": the adapter did not run the law";
    // A fired safety layer or output clamp would silently void the comparison
    // rather than fail it informatively — assert inert, do not assume (S1
    // harness contract item 5).
    ASSERT_FALSE(diag.saturated) << "tick " << tick << ": §10.5 saturation fired";
    ASSERT_FALSE(diag.rate_limited) << "tick " << tick << ": §10.5 rate limit fired";
    if (diag.inertia_clamped)
      ++stats->adapter_clamps;

    const Eigen::VectorXd shim_tau = shim.Step(g, inertia_shaping, state, diag.wrench_lwa);

    for (int j = 0; j < nv; ++j) {
      const auto uj = static_cast<std::size_t>(j);
      const double adapter = out.devices[0].commands[uj];
      ASSERT_LT(std::abs(adapter), rtc::kDefaultMaxJointTorque)
          << "clamp fired at tick " << tick << " ch " << j << " — comparison would be invalid";
      EXPECT_TRUE(BitsEqual(adapter, shim_tau[j]))
          << "tick " << tick << " channel " << j << ": adapter " << adapter << " vs shim "
          << shim_tau[j];
      stats->peak_tau = std::max(stats->peak_tau, std::abs(adapter));
    }

    // REGULATED rows only. `e` is published in full, but under TRANSLATION_ONLY
    // (m = 3) rows 3..5 feed nothing in the §6.2 law — they are the posture
    // task's to drift. Reading all six would let rotational drift alone satisfy
    // "the K_p branch was not inert" while it was inert on every axis the law
    // actually regulates. Same bound the adapter's own §10.6 fault uses
    // (e.head(m), task_impedance_controller.cpp).
    for (int i = 0; i < shim.task_dim(); ++i)
      stats->peak_err =
          std::max(stats->peak_err, std::abs(diag.pose_error[static_cast<std::size_t>(i)]));
  }
}

// One (selection × formulation × Λ_d) combination end to end. `ratio` picks
// whether the §5.2 clamp is inert or engaged, and the caller says which it
// expects — a clamp that fires when the test thought it was inert (or the
// reverse) means the case is not the case it claims to be.
//
// `formulation` runs the GATE. Under kJacobianTranspose the adapter must reach
// §6.2 and stop, so the shim is driven with shaping OFF while the gains still
// carry a Λ_d that would clamp hard: an adapter that shaped anyway diverges from
// the shim bitwise AND trips the clamp count, and neither is silent.
void RunModeCombination(Sel selection, Form formulation, bool natural, double ratio,
                        bool expect_clamp, const std::array<double, 6>& desired_inertia) {
  const int m = (selection == Sel::kFullSe3) ? 6 : 3;
  const bool inertia_shaping = (formulation == Form::kInertiaShaping);
  auto g = CrossCheckGains();
  g.inertia.desired_inertia_natural = natural;
  g.inertia.max_inertia_ratio = ratio;
  g.inertia.desired_inertia = desired_inertia;

  TaskImpedanceController ctrl(Serial7dof(), g, selection);
  ctrl.SetControlRate(1.0 / kDt);
  ctrl.LoadConfig(
      YAML::Load(CrossCheckYaml(inertia_shaping ? "inertia_shaping" : "jacobian_transpose")));
  ctrl.OnDeviceConfigsSet();
  ctrl.set_gains(g);  // LoadConfig owns the wiring; the POD under test comes from here
  ASSERT_EQ(ctrl.formulation(), formulation);
  ASSERT_TRUE(ctrl.external_wrench_enabled());

  ImpedanceShim shim(m);
  ASSERT_EQ(shim.nv(), 7) << "fixture must be redundant (nv > m) for the null-space branch";

  CrossCheckStats stats;
  RunCrossCheck(ctrl, shim, g, inertia_shaping, /*ticks=*/60, &stats);

  EXPECT_GT(stats.peak_tau, 1e-3) << "every compared torque was ~0 — the cross-check is vacuous";
  EXPECT_GT(stats.peak_err, 1e-6) << "the pose error never left zero — the K_p branch was inert";
  EXPECT_GT(shim.wrench_ticks(), 0) << "f_ext was zero on every tick — half the law was not run";
  EXPECT_GT(shim.nullspace_ticks(), 0) << "the null-space branch never ran";
  EXPECT_EQ(shim.solve_failures(), 0) << "Λ_d failed to factorise — that path is oracle 1's, and "
                                         "here it would mean the cross-check compared the DEGRADE";
  if (expect_clamp) {
    EXPECT_GT(shim.clamps(), 0) << "the §5.2 clamp never engaged — this case is meant to be the "
                                   "clamped one";
    EXPECT_EQ(shim.clamps(), stats.adapter_clamps)
        << "shim and adapter disagree on how often the clamp fired";
  } else {
    EXPECT_EQ(shim.clamps(), 0) << "the §5.2 clamp engaged in the case meant to be inert";
    EXPECT_EQ(stats.adapter_clamps, 0) << "the adapter clamped where the shim did not";
  }
}

}  // namespace

// Λ_d well away from Λ_S: B is far from the identity, which is what makes the
// clamped cases clamp. The unclamped shaped case cannot use it — with the bound
// retired, that B drives the joint torque into the ±max_joint_torque saturation
// and a fired safety layer voids a bitwise comparison — so it runs a Λ_d of the
// magnitude a deployment would actually configure.
constexpr std::array<double, 6> kFarInertia{{0.35, 0.35, 0.35, 0.02, 0.02, 0.02}};
constexpr std::array<double, 6> kMildInertia{{1.5, 1.5, 1.5, 0.08, 0.08, 0.08}};

TEST(InertiaShaping, CoreDrivenShimMatchesTheAdapterBitwiseFullSe3Natural) {
  RunModeCombination(Sel::kFullSe3, Form::kInertiaShaping, /*natural=*/true, /*ratio=*/3.0,
                     /*expect_clamp=*/false, kFarInertia);
}

TEST(InertiaShaping, CoreDrivenShimMatchesTheAdapterBitwiseFullSe3Shaped) {
  RunModeCombination(Sel::kFullSe3, Form::kInertiaShaping, /*natural=*/false, /*ratio=*/1.0e9,
                     /*expect_clamp=*/false, kMildInertia);
}

TEST(InertiaShaping, CoreDrivenShimMatchesTheAdapterBitwiseFullSe3ShapedAndClamped) {
  RunModeCombination(Sel::kFullSe3, Form::kInertiaShaping, /*natural=*/false, /*ratio=*/1.05,
                     /*expect_clamp=*/true, kFarInertia);
}

TEST(InertiaShaping, CoreDrivenShimMatchesTheAdapterBitwiseTranslationOnlyShaped) {
  RunModeCombination(Sel::kTranslationOnly, Form::kInertiaShaping, /*natural=*/false,
                     /*ratio=*/1.05, /*expect_clamp=*/true, kFarInertia);
}

TEST(InertiaShaping, CoreDrivenShimMatchesTheAdapterBitwiseTranslationOnlyNatural) {
  RunModeCombination(Sel::kTranslationOnly, Form::kInertiaShaping, /*natural=*/true, /*ratio=*/3.0,
                     /*expect_clamp=*/false, kFarInertia);
}

// The GATE, run closed. Every case above configures kInertiaShaping, so the
// suite's claim that "the shaping is skipped exactly when the adapter skips it"
// (file header) rested on a branch it never took: with `inertia_shaping` pinned
// true at the only call site, an adapter that lost the `&& inertia_shaping`
// conjunct would shape a §6.2 tick and no case here would move.
//
// The gains carry kFarInertia at ratio 1.05 — the very combination the
// FullSe3ShapedAndClamped case uses to force the §5.2 clamp — so a regressed
// gate does not produce a subtle numerical drift but a torque built from a
// hard-clamped B. Two independent assertions catch it: the bitwise comparison
// against a shim that was told NOT to shape, and adapter_clamps, which must stay
// zero because a skipped §6.3 never reaches the clamp at all.
TEST(InertiaShaping, CoreDrivenShimMatchesTheAdapterBitwiseJacobianTransposeSkipsTheShaping) {
  RunModeCombination(Sel::kFullSe3, Form::kJacobianTranspose, /*natural=*/false, /*ratio=*/1.05,
                     /*expect_clamp=*/false, kFarInertia);
}

// The activation ramp is the one piece of wiring that touches the core's INPUT
// rather than its output: f_ext reaches the law as α·f_lwa, and α also scales
// f_task inside the §6.2 helper. With the ramp retired (α = 1 in every case
// above) a binding that forgot to ramp f_ext would still match bit for bit, so
// this case runs the ramp and asserts that partial-α ticks actually occurred.
TEST(InertiaShaping, CoreDrivenShimMatchesTheAdapterBitwiseThroughTheActivationRamp) {
  auto g = CrossCheckGains();
  g.activation_ramp_time = 0.05;  // 25 ticks at 2 ms — partway on most of the run
  g.inertia.desired_inertia_natural = false;
  g.inertia.desired_inertia = kFarInertia;
  g.inertia.max_inertia_ratio = 3.0;

  TaskImpedanceController ctrl(Serial7dof(), g, Sel::kFullSe3);
  ctrl.SetControlRate(1.0 / kDt);
  ctrl.LoadConfig(YAML::Load(CrossCheckYaml("inertia_shaping")));
  ctrl.OnDeviceConfigsSet();
  ctrl.set_gains(g);

  ImpedanceShim shim(6);
  CrossCheckStats stats;
  RunCrossCheck(ctrl, shim, g, /*inertia_shaping=*/true, /*ticks=*/40, &stats);

  EXPECT_GT(stats.peak_tau, 1e-3) << "every compared torque was ~0 — the cross-check is vacuous";
  EXPECT_GT(shim.ramped_ticks(), 0)
      << "α was never strictly between 0 and 1 — the ramp wiring is not being pinned";
  EXPECT_DOUBLE_EQ(shim.alpha(), 1.0) << "the ramp never completed — the run is too short";
}

// `diag.inertia_solve_failed = shaped.solve_failed` is the ONLY path by which an
// operator learns a tick silently ran §6.2 in place of §6.3, and
// docs/compliance-conventions.md makes that distinction normative precisely so a
// numerical breakdown and the §5.2 tuning clamp can be acted on differently.
// Before this case every reference to the field in the repo was an EXPECT_FALSE,
// so deleting the assignment left the whole suite green — the flag stopped being
// reported and nothing moved.
//
// Λ_d is driven non-factorable through set_gains(), which writes the POD
// straight into the SeqLock and bypasses LoadConfig's validation — the same
// bypass the adapter's own NUM-1 floor comment calls out. A negative diagonal
// entry fails the Cholesky at the first pivot.
TEST(InertiaShaping, AdapterReportsTheDegradeWhenLambdaDCannotFactorise) {
  auto g = CrossCheckGains();
  g.inertia.desired_inertia_natural = false;
  g.inertia.max_inertia_ratio = 3.0;
  g.inertia.desired_inertia = {{-1.0, 0.4, 0.4, 0.03, 0.03, 0.03}};  // not positive definite

  TaskImpedanceController ctrl(Serial7dof(), g, Sel::kFullSe3);
  ctrl.SetControlRate(1.0 / kDt);
  ctrl.LoadConfig(YAML::Load(CrossCheckYaml("inertia_shaping")));
  ctrl.OnDeviceConfigsSet();
  ctrl.set_gains(g);

  constexpr int kNv = 7;
  constexpr int kTicks = 20;
  auto state = MakeState(kNv, kDt);
  int degraded_ticks = 0;
  for (int tick = 0; tick < kTicks; ++tick) {
    FillSweep(state, kNv, tick, kDt);
    const auto out = ctrl.Compute(state);
    const auto diag = ctrl.GetDiagnosticsForTesting();
    ASSERT_TRUE(diag.control_valid) << "tick " << tick << ": the degrade must keep the law running";
    EXPECT_FALSE(diag.inertia_clamped)
        << "tick " << tick
        << ": the clamp cannot fire on a tick whose solve never finished — the "
           "two flags have merged";
    if (diag.inertia_solve_failed)
      ++degraded_ticks;
    for (int j = 0; j < kNv; ++j)
      ASSERT_TRUE(std::isfinite(out.devices[0].commands[static_cast<std::size_t>(j)]))
          << "tick " << tick << " ch " << j << ": the degrade emitted a non-finite torque";
  }
  EXPECT_EQ(degraded_ticks, kTicks) << "the adapter never reported the §6.3 degrade — either Λ_d "
                                       "factorised after all, or the flag is not wired";

  // Non-vacuity: the same controller with a factorable Λ_d must CLEAR the flag,
  // or the assertion above would pass just as well on a field wired to a constant.
  g.inertia.desired_inertia = {{0.4, 0.4, 0.4, 0.03, 0.03, 0.03}};
  ctrl.set_gains(g);
  FillSweep(state, kNv, kTicks, kDt);
  static_cast<void>(ctrl.Compute(state));  // the flag, not the torque, is the subject here
  EXPECT_FALSE(ctrl.GetDiagnosticsForTesting().inertia_solve_failed)
      << "a factorable Λ_d still reported the degrade — the flag is stuck, not wired";
}

// ═══════════════════════════════════════════════════════════════════════════
// RT contract
// ═══════════════════════════════════════════════════════════════════════════

// RT-1: the law runs on the tick, and unlike its S1–S3a siblings it is not
// trivially fixed-size — it factorises and solves. The whole point of D-S4a's
// max-size scratch is that this stays heap-free at m < 6, so this gate is the
// sensor for that decision and it takes BOTH halves to be one:
//   • the operator-new counter — sees a std::vector or a header-inline helper
//     (a bare `new`+immediate-`delete` pair would be elided, so a mutation has to
//     be a real runtime-sized allocation reaching an external sink);
//   • the Eigen tripwire — the only one that can see the scratch or its LLT
//     regressing to a runtime-sized Eigen type, which is THE plausible RT-1
//     regression here and is invisible to operator-new interposition because
//     Eigen's internal::aligned_malloc calls std::malloc directly.
// Both are armed by RAII, so an early return cannot leave a live gate behind.
TEST(InertiaShaping, IsAllocationFree) {
  auto rng = MakeRng();

  // Draws are built OUTSIDE the gate: RandomSpd allocates by design (it is the
  // fixture, not the law), and the MatrixXd Λ_S is what the adapter hands over.
  std::vector<CoreDraw> draws;
  draws.reserve(64);
  for (int trial = 1; trial <= 64; ++trial)
    draws.push_back(RandomDraw(rng, trial));

  Vec6 sink = RunCore(draws.front()).f_cmd;  // warm up outside the gate

  std::size_t new_calls = 0;
  std::uint64_t eigen_allocs = 0;
  {
    rtc::testing::ScopedAllocGate heap_gate;
    rtc::testing::ScopedNoMalloc eigen_gate;
    for (const auto& d : draws)
      sink += RunCore(d).f_cmd;
    new_calls = heap_gate.count();
    eigen_allocs = eigen_gate.violations();
  }

  EXPECT_EQ(new_calls, 0u) << "the inertia-shaping law reached operator new on the RT path (RT-1)";
  EXPECT_EQ(eigen_allocs, 0u) << "the inertia-shaping law made Eigen allocate on the RT path — the "
                                 "max-size scratch (D-S4a) has regressed to a dynamic type (RT-1)";
  EXPECT_TRUE(std::isfinite(sink.norm())) << "sink kept the calls from being optimised away";
}
