// ── Segment MPC torque linearisation seam (E1-F01, formulation §1.3) ───────────
// One node's first-order torque model, exposed so a test can check the
// derivative and the offset sign directly instead of only through a QP answer:
//
//   τ(q, q̇, q̈) ≈ τ̄ + D (x − x̄),   D = [∂τ/∂q  ∂τ/∂q̇  M(q̄)]   (n × 3n)
//
// evaluated at x̄ = (q, v, a) with pinocchio::computeRNEADerivatives on the
// given model — armature included when the model carries it (pinocchio adds
// model.armature to τ and to the diagonal of M). Pinocchio fills only the
// upper triangle of M and ADDS the armature to the diagonal of whatever the
// output already holds, so this clears D first and mirrors M — both are part
// of the contract here, not the caller's chore.
//
// RT-safe: no heap, noexcept. The model must have nq == nv.
#pragma once

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace rtc::catching {

/// @param[out] tau τ̄ = RNEA(q, v, a) (n)
/// @param[out] d   D = [∂τ/∂q ∂τ/∂q̇ M] (n × 3n), M symmetric
/// @return false on a size mismatch (outputs untouched).
[[nodiscard]] bool LinearizeTorqueAt(const pinocchio::Model& model, pinocchio::Data& data,
                                     const Eigen::Ref<const Eigen::VectorXd>& q,
                                     const Eigen::Ref<const Eigen::VectorXd>& v,
                                     const Eigen::Ref<const Eigen::VectorXd>& a,
                                     Eigen::Ref<Eigen::VectorXd> tau,
                                     Eigen::Ref<Eigen::MatrixXd> d) noexcept;

}  // namespace rtc::catching
