// ── Golden-vector harness for the §7.3 task-velocity REUSE (#236 S5) ─────────
// Fifth slice of the rtc_controllers pure-library refactor, and the first one
// that adds NO core: TaskAdmittanceController's §7.3 IK velocity term
//
//     ν = ν_c + K^ik ⊙ e
//
// is the SAME velocity-form law S3a already extracted as
// rtc::task::ComputeTaskVelocity, on the same [1/s] gain convention. So S5 is
// design-principles P5 ("generalise instead of forking") applied literally: the
// adapter now calls the existing core, and this file is the sensor for that
// call being inert.
//
// Same contract S1 established and S2a/S3a/S4 followed: the claim under test is
// not "the core is correct" but the stronger "the migration changed NOTHING", so
// every comparison is bitwise (rtc::testing::BitsEqual), never a tolerance.
//
// Two oracles, deliberately:
//
//   1. PreExtractionInlineForm — a literal, rename-only copy of
//      task_admittance_controller.cpp:371-376 at 9a104b00. DURABLE: it outlives
//      the adapter, so it still pins the core after S7 deletes
//      TaskAdmittanceController. It matters here even though the core already
//      has S3a's two oracles, because this controller's inline form was a THIRD
//      shape: it COPIED ν_c and accumulated K⊙e into the copy per element, where
//      the core materialises K⊙e and adds ν_c — the operands the other way
//      round. Addition is commutative in IEEE-754, so the swap is claimed to be
//      bitwise inert; this oracle is where that is MEASURED, on every run,
//      rather than inherited from the session probe that chose D-S5a (plan §S5
//      R3/R4, and the S3a M7 rule: one oracle per inline shape).
//
//   2. The live TaskAdmittanceController. TRANSIENT — RETIRED in #298 S7c-2
//      with the adapter (see "Oracle 2 retired" below). Its job was
//      the one thing oracle 1 structurally cannot do: catch a CORRELATED
//      transcription error, plus the WIRING — that ν_c and e reach the core in
//      that order and in that role, that the gains marshalled into TaskVelParams
//      are ik_kp_pos/ik_kp_rot and not some other pair, and that the law still
//      runs in every mode combination the adapter has.
//
// Observability here is better than S2a's or S4's and matches S3a's: ν leaves
// the law through ik_.Solve into dq_ → target_velocities, and its integral is
// commands. The shim therefore does NOT re-derive the compliant frame or the
// pseudoinverse — it instantiates the adapter's own public helpers
// (compliance::AdmittanceIntegrator, compliance::DifferentialIk) and compares
// both device-0 lanes directly (plan §S5 R5). What it does replicate is the
// JOINT TAIL (velocity clamp → integrate → joint-limit clamp → rate re-bound →
// dq recompute), because target_velocities is the round-tripped
// (q_cmd − q_base)/dt and not the raw solve output. That tail is limits-and-
// policy, not a law: R6(a) — it is NOT merged with CLIK's, and it stays the
// binding's (S7).
//
// The one lane the shim BORROWS instead of replicating is the conditioned
// wrench (diag.wrench_lwa), exactly as S4 did: the §3.2.1 pipeline is upstream
// of the law and out of scope for this slice. Everything downstream of the
// borrow — the α scaling, the integrator, the error, the law, the IK — is the
// shim's own.
//
// The shim reads J from a SECOND, independently built RtModelHandle. That
// premise is measured rather than assumed, and this file reuses BOTH halves of
// the existing measurement (same serial_7dof fixture, same handle path, plan
// §S3 R6): the dynamics half by ReferenceDynamics.IndependentHandlesAgreeBitwise
// (test_task_accel_core.cpp) and the reorder half — which this shim also
// exercises, on the posture gather and both scatters — by
// ReferenceHandles.IndependentHandlesAgreeOnTheReorderPathBitwise
// (test_task_vel_core.cpp).
//
// The bitwise suite carries a non-vacuity partner
// (BitwiseComparisonRejectsAFusedMultiplyAdd) proving the comparison can reject
// a different ALGORITHM for the same expression — not merely a different
// spelling of the same one, which the S4 probe showed proves nothing.

// no_malloc_scope.hpp MUST precede every Eigen header — it installs the Eigen
// allocation tripwire by defining EIGEN_RUNTIME_NO_MALLOC and its own
// eigen_assert, and a later include would be silently ignored (its own #error
// enforces the ordering). Everything below pulls Eigen, so it comes first.
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_controllers/compliance/admittance_integrator.hpp"
#include "rtc_controllers/compliance/differential_ik.hpp"
#include "rtc_controllers/params/task_admittance_params.hpp"
#include "rtc_controllers/task/task_vel_law.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/bit_compare.hpp"
#include "rtc_controllers/testing/serial7dof_fixture.hpp"
#include "rtc_math/se3/pinocchio_adapter.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <random>
#include <span>
#include <vector>

// The allocation gate is TWO complementary sensors, and this call site needs
// both:
//   • rtc::testing::ScopedAllocGate (rtc_controllers/testing/alloc_gate.hpp)
//     counts `operator new`, so it sees a std::vector or a header-inline helper.
//   • rtc::testing::ScopedNoMalloc (rtc_base/testing/no_malloc_scope.hpp) rides
//     eigen_assert, so it sees EIGEN allocation — which the first gate CANNOT.
// IsAllocationFree arms both.

namespace {

using Vec6 = Eigen::Matrix<double, 6, 1>;

using rtc::kMaxDeviceChannels;
using rtc::task::ComputeTaskVelocity;
using rtc::task::TaskVelParams;
using rtc::testing::BitsEqual;
using rtc::testing::FillSweep;
using rtc::testing::MakeHandle;
using rtc::testing::MakeRng;
using rtc::testing::MakeState;
using rtc::testing::Serial7dof;

constexpr double kDt = 0.002;

// Mirrors the adapter's own anonymous-namespace constant (NUM-1 floor on λ_max,
// task_admittance_controller.cpp). Copied rather than exported because the
// floor is the ADAPTER's decision about its own YAML surface, not part of the
// law under test — the same call S4's shim made.
constexpr double kMinMaxDamping = 1e-4;

// ═══════════════════════════════════════════════════════════════════════════
// Oracle 1 — the literal pre-extraction form
// ═══════════════════════════════════════════════════════════════════════════

// One random tick's worth of core inputs, in the shape the ADAPTER holds them:
// ν_c is the compliant-frame velocity (a state, not a trajectory sample) and e
// is the pose error the binding computes.
struct CoreDraw {
  std::array<double, 3> ik_kp_pos{};
  std::array<double, 3> ik_kp_rot{};
  Vec6 pose_err{Vec6::Zero()};
  Vec6 nu_c{Vec6::Zero()};
};

// EXACTLY the expression as it stood inline in
// TaskAdmittanceController::Compute() before #236 S5 routed it through
// task/task_vel_law.hpp (task_admittance_controller.cpp:371-376 at 9a104b00).
// Rename-only transcription:
//   gains.ik_kp_pos → d.ik_kp_pos   gains.ik_kp_rot → d.ik_kp_rot
//   e               → d.pose_err    nu_c            → d.nu_c
// Nothing else moved: same operation order, same association — ν_c is COPIED
// and each axis accumulates its own K·e into that copy, three axes per loop
// iteration. This is the shape whose equivalence to the core's
// (K⊙e materialised, then += ν_ff) is the whole claim of S5.
Vec6 PreExtractionInlineForm(const CoreDraw& d) {
  Eigen::Matrix<double, 6, 1> nu = d.nu_c;
  for (int i = 0; i < 3; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    nu(i) += d.ik_kp_pos[ui] * d.pose_err(i);
    nu(i + 3) += d.ik_kp_rot[ui] * d.pose_err(i + 3);
  }
  return nu;
}

// Drive the core with the same draw — this is the marshalling the adapter now
// does, ν_c in the feedforward slot.
Vec6 RunCore(const CoreDraw& d) {
  return ComputeTaskVelocity(TaskVelParams{d.ik_kp_pos, d.ik_kp_rot}, d.pose_err, d.nu_c);
}

// Draws sweep the magnitudes this law actually meets. ik_kp defaults to 2.0
// [1/s] and is O(1); the pose error is small once the arm tracks the compliant
// frame and O(0.1 m) right after a push; ν_c is bounded by the §7.5 velocity
// guard (0.25 m/s linear, 0.8 rad/s angular by default) but can dominate at the
// start of a contact. Every few trials a term is zeroed or made near-cancelling
// so no axis is permanently masked by a larger one — a cancellation that only
// shows up when the terms are comparable is precisely what a regrouping
// changes, and it is also the only regime where the ORDER of the two operands
// could plausibly matter.
CoreDraw RandomDraw(std::mt19937& rng, int trial) {
  std::uniform_real_distribution<double> gain(0.1, 12.0);
  std::normal_distribution<double> err(0.0, 0.2);
  std::normal_distribution<double> vel(0.0, 0.4);

  CoreDraw d;
  for (std::size_t i = 0; i < 3; ++i) {
    d.ik_kp_pos[i] = gain(rng);
    d.ik_kp_rot[i] = gain(rng);
  }
  for (int i = 0; i < 6; ++i) {
    d.pose_err[i] = err(rng);
    d.nu_c[i] = vel(rng);
  }

  switch (trial % 5) {
    case 1:
      d.pose_err.setZero();  // tracking converged — the feedforward alone
      break;
    case 2:
      d.nu_c.setZero();  // compliant frame at rest — the CLIK term alone
      break;
    case 3:
      // §7.3 read literally: ik_kp = 0 makes ν = ν_c, the pure-feedforward form
      // the spec states and the gains doc calls out as reproducible.
      d.ik_kp_pos = {{0.0, 0.0, 0.0}};
      d.ik_kp_rot = {{0.0, 0.0, 0.0}};
      break;
    case 4:
      // Near-cancelling: ν_c almost exactly opposes the P term on all six axes,
      // so the sum is a catastrophic subtraction.
      for (int i = 0; i < 3; ++i) {
        const auto ui = static_cast<std::size_t>(i);
        d.nu_c[i] = -d.ik_kp_pos[ui] * d.pose_err[i];
        d.nu_c[i + 3] = -d.ik_kp_rot[ui] * d.pose_err[i + 3];
      }
      break;
    default:
      break;
  }
  return d;
}

}  // namespace

TEST(AdmittanceTaskVelReuse, MatchesThePreExtractionInlineFormBitwise) {
  auto rng = MakeRng();
  double peak = 0.0;
  int zero_gain_trials = 0;

  for (int trial = 0; trial < 600; ++trial) {
    const CoreDraw d = RandomDraw(rng, trial);

    const Vec6 got = RunCore(d);
    const Vec6 want = PreExtractionInlineForm(d);

    for (int i = 0; i < 6; ++i) {
      EXPECT_TRUE(BitsEqual(got[i], want[i]))
          << "trial " << trial << " axis " << i << ": " << got[i] << " vs " << want[i];
      peak = std::max(peak, std::abs(got[i]));
    }
    if (d.ik_kp_pos[0] == 0.0 && d.ik_kp_rot[0] == 0.0)
      ++zero_gain_trials;
  }

  EXPECT_GT(peak, 1e-6) << "every compared axis was ~0 — the draw generator degenerated";
  // The §7.3-literal operating point is a branch of the DRAW, not of the code,
  // and a draw generator that stopped producing it would quietly shrink the
  // claim to "the law agrees where the P term is live".
  EXPECT_GT(zero_gain_trials, 0) << "the pure-feedforward (K^ik = 0) draws never ran";
}

// Non-vacuity partner, and NOT a different spelling of the same algorithm — the
// S4 probe's lesson (an out-of-place solve turned out bit-identical to
// solveInPlace, so a mutant built that way would have proved nothing). This one
// contracts each axis with std::fma, which computes kp·e + ν_c in ONE rounding
// instead of two: algebraically the same value, a genuinely different
// algorithm, and the same control the D-S5a probe used (622,794 of 1.2M doubles
// differed). If bit equality could not tell these apart, the test above would be
// pinning nothing — in particular it could not distinguish the operand swap it
// exists to certify from a compiler that fused the expression.
TEST(AdmittanceTaskVelReuse, BitwiseComparisonRejectsAFusedMultiplyAdd) {
  auto rng = MakeRng();
  int differing_axes = 0;
  int compared = 0;

  for (int trial = 0; trial < 600; ++trial) {
    const CoreDraw d = RandomDraw(rng, trial);
    const Vec6 got = RunCore(d);

    for (int i = 0; i < 6; ++i) {
      const double kp = (i < 3) ? d.ik_kp_pos[static_cast<std::size_t>(i)]
                                : d.ik_kp_rot[static_cast<std::size_t>(i - 3)];
      const double fused = std::fma(kp, d.pose_err[i], d.nu_c[i]);

      ++compared;
      if (!BitsEqual(got[i], fused))
        ++differing_axes;
      EXPECT_NEAR(got[i], fused, 1e-6 * (1.0 + std::abs(got[i])))
          << "the mutant must stay algebraically equivalent, else it proves nothing";
    }
  }

  ASSERT_GT(compared, 0) << "no axes compared — the draw generator degenerated";
  EXPECT_GT(differing_axes, 0)
      << "bit comparison cannot distinguish a fused multiply-add — the equivalence test is vacuous";
}

// ═══════════════════════════════════════════════════════════════════════════
// Oracle 2 — the shim (prototype of the S7 integration-layer binding)
// ═══════════════════════════════════════════════════════════════════════════

namespace {

// What one tick of the adapter puts on device 0, in device channel order, plus
// the two core INPUTS the adapter also publishes — pinning those separately is
// what localises a failure to the binding (error definition, compliant frame)
// rather than leaving the whole tick suspect.
struct ShimOutput {
  std::array<double, kMaxDeviceChannels> target_velocities{};
  std::array<double, kMaxDeviceChannels> commands{};
  Vec6 pose_error{Vec6::Zero()};
  Vec6 nu_c{Vec6::Zero()};
  /// The posture gate this tick, as the shim evaluated it. Compared against the
  /// adapter's diag.nullspace_active because the gate is otherwise NUMERICALLY
  /// INERT when closed: with nullspace_kp = 0 the ungated path scatters
  /// `0.0 · Δq` — signed zeros that N projects and the solve then adds as an
  /// exact no-op. Measured, not argued: dropping `&& nullspace_kp != 0.0` from
  /// the adapter left every bitwise lane in this suite green until this field
  /// existed (the S4 (0b) lesson, in its diagnostic-flag form).
  bool nullspace_active{false};
};

// The adapter's TaskAdmittanceController::kTaskDim, kept local now that the
// adapter is gone. 6-D task space is the law's own shape (a full SE(3) twist),
// not a configurable, so a literal here is the honest form.
constexpr int kShimTaskDim = 6;

// Owns the model, the compliant-frame integrator, the differential IK and the
// joint tail (STRUCTURE) and calls the core (ALGORITHM). Mirrors
// TaskAdmittanceController::Compute() step for step from the joint-state copy
// down to the emitted lanes; any divergence here is a divergence the future
// binding would ship.
//
// Unlike S2a's and S4's shims this one does NOT re-derive the helpers it drives:
// AdmittanceIntegrator and DifferentialIk are public, so it instantiates the
// same classes the adapter holds and only the plumbing between them is written
// twice (plan §S5 R5).
class AdmittanceShim {
 public:
  AdmittanceShim() {
    handle_ = MakeHandle(model_);
    tip_ = static_cast<pinocchio::FrameIndex>(model_->nframes - 1);
    nv_ = handle_->nv();

    J_full_ = Eigen::MatrixXd::Zero(6, nv_);
    dq_ = Eigen::VectorXd::Zero(nv_);
    dq_dev_ = Eigen::VectorXd::Zero(nv_);
    qdot_null_ = Eigen::VectorXd::Zero(nv_);
    qdot_null_dev_ = Eigen::VectorXd::Zero(nv_);
    q_null_ = Eigen::VectorXd::Zero(nv_);
    q_dev_ = Eigen::VectorXd::Zero(nv_);
    desired_q_ = Eigen::VectorXd::Zero(nv_);
    q_base_ = Eigen::VectorXd::Zero(nv_);
    ik_.Resize(nv_, kShimTaskDim);
  }

  [[nodiscard]] int nv() const { return nv_; }

  // One tick. `f_lwa` is the conditioned external wrench the adapter produced on
  // THIS tick (diag.wrench_lwa) — the one lane the shim borrows rather than
  // replicates; see the file header.
  ShimOutput Step(const rtc::params::TaskAdmittanceParams& g, const rtc::ControllerState& state,
                  const std::array<double, 6>& f_lwa) {
    const auto& dev0 = state.devices[0];
    const double dt = state.dt;

    std::array<double, kMaxDeviceChannels> q_buf{};
    for (int i = 0; i < nv_; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      q_buf[ui] = dev0.positions[ui];
      q_dev_(i) = dev0.positions[ui];
    }
    std::span<const double> q_span(q_buf.data(), static_cast<std::size_t>(nv_));

    handle_->ComputeJacobians(q_span);
    handle_->GetFrameJacobian(tip_, pinocchio::LOCAL_WORLD_ALIGNED, J_full_);
    const pinocchio::SE3& tcp = handle_->GetFramePlacement(tip_);

    // Seed X_d, the posture target and the command from the measurement, as the
    // adapter's !target_initialized_ branch does (§10.7). The integrator Reset
    // is part of that seed: an activation must not inherit a deviation.
    if (!initialized_) {
      goal_pose_ = tcp;
      for (int i = 0; i < nv_; ++i) {
        q_null_(i) = q_dev_(i);
        desired_q_(i) = q_dev_(i);
      }
      integrator_.Reset();
      elapsed_ = 0.0;
      initialized_ = true;
    }

    const double ramp = g.activation_ramp_time;
    alpha_ = (ramp <= 0.0) ? 1.0 : std::min(1.0, elapsed_ / ramp);
    if (alpha_ > 0.0 && alpha_ < 1.0)
      ++ramped_ticks_;

    Vec6 f_raw;
    for (int i = 0; i < 6; ++i)
      f_raw(i) = f_lwa[static_cast<std::size_t>(i)];
    if (f_raw.norm() > 0.0)
      ++wrench_ticks_;
    const Vec6 f_ext = alpha_ * f_raw;

    // ── §7.2 compliant frame ───────────────────────────────────────────────
    const rtc::compliance::AdmittanceStatus ast = integrator_.Step(g.admittance, f_ext, dt);
    if (!ast.finite)
      ++nonfinite_steps_;
    compliant_pose_.rotation() = integrator_.rotation() * goal_pose_.rotation();
    compliant_pose_.translation() = goal_pose_.translation() + integrator_.translation();
    const Vec6& nu_c = integrator_.velocity();

    // ── §7.3: the error DEFINITION is the binding's, the law is the core's ──
    const Vec6 e = rtc::math::se3::computePoseError(tcp, compliant_pose_,
                                                    rtc::math::se3::ErrorType::SplitWorld);
    const Vec6 nu = ComputeTaskVelocity(TaskVelParams{g.ik_kp_pos, g.ik_kp_rot}, e, nu_c);

    const bool nullspace_active = (nv_ > kShimTaskDim) && (g.nullspace_kp != 0.0);
    if (nullspace_active) {
      for (int i = 0; i < nv_; ++i)
        qdot_null_dev_(i) = g.nullspace_kp * (q_null_(i) - q_dev_(i));
      handle_->ReorderInput(
          std::span<const double>(qdot_null_dev_.data(), static_cast<std::size_t>(nv_)),
          qdot_null_);
      ++nullspace_ticks_;
      peak_null_ = std::max(peak_null_, qdot_null_dev_.cwiseAbs().maxCoeff());
    } else {
      qdot_null_.setZero();
    }

    const double max_damping = std::max(kMinMaxDamping, g.max_damping);
    const rtc::compliance::DifferentialIk::Result ik =
        ik_.Compute(J_full_, g.singularity_threshold, max_damping);
    if (ik.ok) {
      ik_.Solve(nu, qdot_null_, dq_);
    } else {
      dq_.setZero();
      ++ik_failures_;
    }

    // ── The joint TAIL (adapter :404-464). Limits and policy, not a law —
    //     R6(a): replicated here, NOT merged with CLIK's, and left to the S7
    //     binding. Reproduced verbatim because target_velocities is the
    //     ROUND-TRIPPED (q_cmd − q_base)/dt, which is not bit-identical to the
    //     solve output it came from. ────────────────────────────────────────
    const auto nvz = static_cast<std::size_t>(nv_);
    handle_->ReorderOutput(dq_, std::span<double>(dq_dev_.data(), nvz));
    for (int i = 0; i < nv_; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      const double lim = (ui < max_joint_velocity_.size()) ? max_joint_velocity_[ui]
                                                           : rtc::kDefaultMaxJointVelocity;
      if (dq_dev_(i) > lim) {
        dq_dev_(i) = lim;
        ++vel_clamps_;
      } else if (dq_dev_(i) < -lim) {
        dq_dev_(i) = -lim;
        ++vel_clamps_;
      }
    }

    for (int i = 0; i < nv_; ++i) {
      q_base_(i) = g.integrate_from_measured ? q_dev_(i) : desired_q_(i);
      desired_q_(i) = q_base_(i) + dq_dev_(i) * dt;
    }

    ShimOutput out;
    for (int i = 0; i < nv_; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      const double lo = position_lower_[ui] + g.joint_limit_margin;
      const double hi = position_upper_[ui] - g.joint_limit_margin;
      if (lo < hi)
        desired_q_(i) = std::clamp(desired_q_(i), lo, hi);

      const double lim = (ui < max_joint_velocity_.size()) ? max_joint_velocity_[ui]
                                                           : rtc::kDefaultMaxJointVelocity;
      const double step = std::abs(lim) * dt;
      const double bounded = std::clamp(desired_q_(i), q_base_(i) - step, q_base_(i) + step);
      if (bounded != desired_q_(i)) {
        desired_q_(i) = bounded;
        ++rate_rebounds_;
      }
      dq_dev_(i) = (desired_q_(i) - q_base_(i)) / dt;

      out.commands[ui] = desired_q_(i);
      out.target_velocities[ui] = dq_dev_(i);
    }
    out.pose_error = e;
    out.nu_c = nu_c;
    // The adapter reports `nullspace_active && ik.ok`; ik_failures_ is asserted
    // zero by the caller, so the two agree exactly when the gates do.
    out.nullspace_active = nullspace_active && ik.ok;

    elapsed_ += dt;  // the adapter advances the ramp only on a clean control tick
    return out;
  }

  // Branch counters — a cross-check that never took the branch it was written
  // for is vacuous with respect to the state machine it exists to pin (the S2a
  // lesson: the π-split test was green with splits() == 0 before this
  // instrumentation existed).
  [[nodiscard]] int nullspace_ticks() const { return nullspace_ticks_; }

  [[nodiscard]] double peak_null() const { return peak_null_; }

  [[nodiscard]] int wrench_ticks() const { return wrench_ticks_; }

  [[nodiscard]] int ramped_ticks() const { return ramped_ticks_; }

  [[nodiscard]] int vel_clamps() const { return vel_clamps_; }

  [[nodiscard]] int rate_rebounds() const { return rate_rebounds_; }

  [[nodiscard]] int ik_failures() const { return ik_failures_; }

  [[nodiscard]] int nonfinite_steps() const { return nonfinite_steps_; }

 private:
  std::shared_ptr<const pinocchio::Model> model_;
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle_;
  pinocchio::FrameIndex tip_{0};
  int nv_{0};

  rtc::compliance::AdmittanceIntegrator integrator_;
  rtc::compliance::DifferentialIk ik_;

  bool initialized_{false};
  double elapsed_{0.0};
  double alpha_{0.0};
  int nullspace_ticks_{0};
  int wrench_ticks_{0};
  int ramped_ticks_{0};
  int vel_clamps_{0};
  int rate_rebounds_{0};
  int ik_failures_{0};
  int nonfinite_steps_{0};
  double peak_null_{0.0};

  pinocchio::SE3 goal_pose_{pinocchio::SE3::Identity()};
  pinocchio::SE3 compliant_pose_{pinocchio::SE3::Identity()};
  Eigen::MatrixXd J_full_;
  Eigen::VectorXd dq_;
  Eigen::VectorXd dq_dev_;
  Eigen::VectorXd qdot_null_;
  Eigen::VectorXd qdot_null_dev_;
  Eigen::VectorXd q_null_;
  Eigen::VectorXd q_dev_;
  Eigen::VectorXd desired_q_;
  Eigen::VectorXd q_base_;

  // The adapter's own inert defaults (InitFromModel), which OnDeviceConfigsSet
  // leaves in place when no device config carries joint limits — the regime
  // every case below runs in. Kept as vectors so the tail reads exactly as the
  // adapter's does.
  std::vector<double> max_joint_velocity_ = std::vector<double>(
      static_cast<std::size_t>(rtc::kMaxRobotDOF), rtc::kDefaultMaxJointVelocity);
  std::vector<double> position_lower_ =
      std::vector<double>(static_cast<std::size_t>(rtc::kMaxRobotDOF), -1e9);
  std::vector<double> position_upper_ =
      std::vector<double>(static_cast<std::size_t>(rtc::kMaxRobotDOF), 1e9);
};

// The reference gains oracle 2 was driven with, kept for the RT gate below.
// Six PAIRWISE-DISTINCT gains, not two isotropic blocks: oracle 2 existed to
// catch WIRING errors, and a permutation inside one block — ik_kp_pos marshalled
// as {y,x,z} into TaskVelParams — is bit-for-bit invisible when all three values
// are equal. The distinctness still earns its keep for the allocation gate,
// which reads the same POD.
rtc::params::TaskAdmittanceParams CrossCheckGains() {
  rtc::params::TaskAdmittanceParams g;
  g.ik_kp_pos = {{2.0, 1.4, 1.7}};
  g.ik_kp_rot = {{1.5, 0.9, 1.2}};
  g.nullspace_kp = 0.5;
  g.activation_ramp_time = 0.0;
  g.joint_limit_margin = 0.0;
  return g;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Oracle 2 retired with the adapter (#298 S7c-2)
// ═══════════════════════════════════════════════════════════════════════════
//
// Seven cross-check cases stood here, driving the live TaskAdmittanceController
// and AdmittanceShim through the same tick sequence and comparing
// target_velocities, the integrated commands, the compliant-frame velocity and
// the pose error bit for bit:
//
//   AdmittanceTaskVelReuse.CoreDrivenShimMatchesTheAdapterBitwiseMeasuredBaseWithNullSpace
//   AdmittanceTaskVelReuse.CoreDrivenShimMatchesTheAdapterBitwiseMeasuredBaseWithoutNullSpace
//   AdmittanceTaskVelReuse.CoreDrivenShimMatchesTheAdapterBitwiseSelfIntegratingWithNullSpace
//   AdmittanceTaskVelReuse.CoreDrivenShimMatchesTheAdapterBitwiseSelfIntegratingWithoutNullSpace
//   AdmittanceTaskVelReuse.CoreDrivenShimMatchesTheAdapterBitwiseWithTheSpecLiteralZeroIkGain
//   AdmittanceTaskVelReuse.CoreDrivenShimMatchesTheAdapterBitwiseWithTheSpecLiteralZeroIkGainSelfIntegrating
//   AdmittanceTaskVelReuse.CoreDrivenShimMatchesTheAdapterBitwiseThroughTheActivationRamp
//
// VERIFICATION PROVENANCE — the task::ComputeTaskVelocity REUSE and
// PreExtractionInlineForm above were verified bitwise against the live
// TaskAdmittanceController @ 0f873901 over 40 ticks x 7 channels across
// integrate_from_measured x nullspace_kp x ik_kp, including the spec-literal
// K^ik = 0 operating point in both integration modes, plus the activation ramp
// with partial-alpha ticks asserted to have occurred. Oracle 2 retired in S7c.
//
// They could not be migrated: oracle 2's unique job was the CORRELATED
// transcription error, which by construction needs the shipped code to compare
// against. See test_joint_pd_core.cpp's equivalent block for the full argument.
//
// AdmittanceShim above is kept as the reference recipe an S7 binding starts
// from — it instantiates the adapter's own AdmittanceIntegrator / DifferentialIk
// rather than re-deriving them. Nothing pins it any more.

// ═══════════════════════════════════════════════════════════════════════════
// RT contract
// ═══════════════════════════════════════════════════════════════════════════

// RT-1: the law runs on the tick. test_task_vel_core.cpp already gates the CORE
// itself; what is gated HERE is the ADAPTER'S marshalling of it — the
// TaskVelParams built from the Gains arrays on every tick and the Vec6 handed in
// as ν_ff. The adapter's own ComputeIsAllocationFree
// (test_task_admittance_controller.cpp) cannot see the Eigen half of that:
// Compute() lives in another TU, where the eigen_assert tripwire this file
// installs is not compiled in.
//
// It takes BOTH sensors to be one gate:
//   • the operator-new counter — sees a std::vector or a header-inline helper
//     (a bare `new`+immediate-`delete` pair would be elided, so a mutation has
//     to be a real runtime-sized allocation reaching an external sink);
//   • the Eigen tripwire — the only one that can see a fixed-size argument or
//     temporary regressing to a runtime-sized Eigen type, which is this call
//     site's most plausible RT-1 regression and is invisible to operator-new
//     interposition because Eigen's internal::aligned_malloc calls std::malloc
//     directly.
// Both are armed by RAII, so an early return cannot leave a live gate behind.
TEST(AdmittanceTaskVelReuse, IsAllocationFree) {
  auto rng = MakeRng();

  const CoreDraw warm = RandomDraw(rng, 0);
  Vec6 sink = RunCore(warm);  // warm up outside the gate

  std::vector<CoreDraw> draws;
  draws.reserve(64);
  for (int trial = 1; trial <= 64; ++trial)
    draws.push_back(RandomDraw(rng, trial));

  // The controller-level gains POD, rather than a bare TaskVelParams: a
  // regression that made its ik_kp_* something other than a std::array would
  // then show up here as an allocation on the way in.
  rtc::params::TaskAdmittanceParams g = CrossCheckGains();

  std::size_t new_calls = 0;
  std::uint64_t eigen_allocs = 0;
  {
    rtc::testing::ScopedAllocGate heap_gate;
    rtc::testing::ScopedNoMalloc eigen_gate;
    for (const auto& d : draws) {
      g.ik_kp_pos = d.ik_kp_pos;
      g.ik_kp_rot = d.ik_kp_rot;
      sink += ComputeTaskVelocity(TaskVelParams{g.ik_kp_pos, g.ik_kp_rot}, d.pose_err, d.nu_c);
    }
    new_calls = heap_gate.count();
    eigen_allocs = eigen_gate.violations();
  }

  EXPECT_EQ(new_calls, 0u)
      << "the §7.3 velocity call site reached operator new on the RT path (RT-1)";
  EXPECT_EQ(eigen_allocs, 0u)
      << "the §7.3 velocity call site made Eigen allocate on the RT path (RT-1)";
  EXPECT_TRUE(std::isfinite(sink.norm())) << "sink kept the calls from being optimised away";
}
