// ── Kinematic differential IK for compliance controllers (spec §7.3 방식 1) ──
// Shared numerical core for turning a task twist into a joint-velocity command:
//
//     q̇ = J_S⁺(ν_d + ν_c) + (I − J_S⁺J_S)·q̇_null,
//     J_S⁺ = J_Sᵀ(J_S J_Sᵀ + λ²I)⁻¹                        (damped least squares)
//
// Pure Eigen, RT-safe after Resize() (noexcept, no heap, no throw).
//
// ── Why a new helper and not one of the two DLS blocks already in the repo ──
// There are two, and they are DIFFERENT FAMILIES that must stay distinguishable
// (issue #236 D16):
//   • ClikController inlines a CONSTANT-λ DLS. Reusing it means copying it —
//     it is not a callable unit — and a copy would be the THIRD variant, which
//     is exactly the fork design-principles P5 forbids.
//   • compliance/task_dynamics.hpp is the DYNAMICALLY-CONSISTENT inverse
//     J̄_S = M⁻¹J_SᵀΛ_S. It needs the joint inertia M and produces a TORQUE-space
//     nullspace projector Nᵀ. Admittance emits positions and has no business
//     forming M, so it is the wrong object here — but its σ_min definition and
//     §6.5 adaptive-λ rule ARE shared, and are called, not re-derived.
//
// So this class is the kinematic sibling: same §6.5 damping law, velocity-space
// projector N = I − J⁺J (NOT the transpose form — that one projects torques).
//
// CLIK's migration onto this helper is deliberately NOT part of this slice, the
// same call task_dynamics.hpp made about the OSC when it landed (#236 D16): a
// helper extraction and a behavioural migration of a shipped controller are
// separate commits with separate regression surfaces (PROC-6).
#pragma once

#include "rtc_controllers/compliance/task_dynamics.hpp"

#include <Eigen/Cholesky>  // LLT
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace rtc::compliance {

/// Pre-sized buffers so Solve()/Compute() are heap-free on the RT path.
/// Resize() is off-RT (lifecycle) and allocates; nothing else may run before it.
class DifferentialIk {
 public:
  struct Result {
    double sigma_min{0.0};  ///< σ_min(J_S) — kinematic distance to singularity
    double lambda_sq{0.0};  ///< §6.5 DLS damping λ² applied (0 = far from singular)
    bool ok{false};         ///< false ⇒ J⁺/N are STALE; the caller must not use them
  };

  /// Size for an nv-DoF model and an m-dimensional task. Off-RT only.
  void Resize(int nv, int m) {
    nv_ = nv;
    m_ = m;
    J_ = Eigen::MatrixXd::Zero(m, nv);
    JJt_ = Eigen::MatrixXd::Zero(m, m);
    A_ = Eigen::MatrixXd::Zero(m, m);
    AinvJ_ = Eigen::MatrixXd::Zero(m, nv);
    Jpinv_ = Eigen::MatrixXd::Zero(nv, m);
    N_ = Eigen::MatrixXd::Identity(nv, nv);
    llt_A_ = Eigen::LLT<Eigen::MatrixXd>(m);
    saes_ = Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(m);
  }

  /// Form J⁺ and N from the selected Jacobian J_S (m×nv). `sigma0` / `lambda_max`
  /// parameterise the §6.5 σ_min-adaptive damping — identical rule to the
  /// dynamically-consistent path, called rather than duplicated.
  ///
  /// A false `ok` means the m×m normal-equation matrix would not factor. Note
  /// what does NOT trigger it: J J^T is positive SEMI-definite for any real J,
  /// and Eigen's LLT accepts a singular one, so a rank-deficient pose reports
  /// ok WITH σ_min ≈ 0 — the σ_min faults, not this flag, are what catch it. The
  /// reachable failure is a non-finite J (a NaN joint state propagating through
  /// FK). J⁺ and N then keep their previous contents on purpose: overwriting
  /// them would let a caller that ignores `ok` command a NaN joint velocity
  /// instead of the last good inverse.
  [[nodiscard]] Result Compute(const Eigen::Ref<const Eigen::MatrixXd>& J_S, double sigma0,
                               double lambda_max) noexcept {
    Result r;
    // Checked explicitly because NOTHING downstream does: Eigen's LLT reports
    // Success on a matrix full of NaN (every "is this pivot positive" test is a
    // comparison, and comparisons against NaN are false), so without this a NaN
    // joint state would sail through and be published as a joint command.
    if (!J_S.allFinite())
      return r;  // r.ok stays false; J⁺/N keep the last good contents
    J_ = J_S;
    r.sigma_min = SmallestSingularValue(J_, JJt_, saes_);
    r.lambda_sq = AdaptiveDampingSquared(r.sigma_min, sigma0, lambda_max);

    // A = J Jᵀ + λ²I. JJt_ already holds J Jᵀ (SmallestSingularValue wrote it).
    A_ = JJt_;
    A_.diagonal().array() += r.lambda_sq;
    llt_A_.compute(A_);
    if (llt_A_.info() != Eigen::Success)
      return r;  // r.ok stays false

    // J⁺ = Jᵀ A⁻¹ = (A⁻¹ J)ᵀ  — A is symmetric, so one solve gives both.
    AinvJ_ = J_;
    llt_A_.solveInPlace(AinvJ_);  // A⁻¹ J   (m×nv)
    Jpinv_.noalias() = AinvJ_.transpose();

    // Velocity-space nullspace projector. N (not Nᵀ): q̇_null is a VELOCITY, and
    // the transpose form is the one that belongs on torques (§6.4).
    N_.setIdentity();
    N_.noalias() -= Jpinv_ * J_;

    r.ok = true;
    return r;
  }

  /// q̇ = J⁺·twist + N·q̇_null. `twist` is m-dimensional in the same frame J_S was
  /// expressed in; `qdot_null` is nv (pass a zero vector for no posture task).
  /// Requires a preceding Compute() that returned ok.
  void Solve(const Eigen::Ref<const Eigen::VectorXd>& twist,
             const Eigen::Ref<const Eigen::VectorXd>& qdot_null,
             Eigen::Ref<Eigen::VectorXd> qdot_out) const noexcept {
    qdot_out.noalias() = Jpinv_ * twist;
    qdot_out.noalias() += N_ * qdot_null;
  }

  [[nodiscard]] const Eigen::MatrixXd& PseudoInverse() const noexcept { return Jpinv_; }

  [[nodiscard]] const Eigen::MatrixXd& NullspaceProjector() const noexcept { return N_; }

  [[nodiscard]] int nv() const noexcept { return nv_; }

  [[nodiscard]] int m() const noexcept { return m_; }

 private:
  int nv_{0};
  int m_{0};
  Eigen::MatrixXd J_;
  Eigen::MatrixXd JJt_;
  Eigen::MatrixXd A_;
  Eigen::MatrixXd AinvJ_;
  Eigen::MatrixXd Jpinv_;
  Eigen::MatrixXd N_;
  Eigen::LLT<Eigen::MatrixXd> llt_A_;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes_;
};

}  // namespace rtc::compliance
