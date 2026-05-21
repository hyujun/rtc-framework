#pragma once

#include "rtc_tsid/types/wbc_types.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Stage B-2: GraspCache — RT-safe pseudo-inverses + nullspace projector of
// the grasp matrix G ∈ R^{6 × n_λ}.
//
// Quantities cached per tick:
//   G_pinv   ∈ R^{n_λ × 6}    — pseudo-inverse, used by InternalForceTask
//                                 reference projection (Stage B-3) and as
//                                 the right inverse satisfying G·G_pinv = I
//                                 when G is row-full-rank.
//   GT_pinv  ∈ R^{6 × n_λ}    — pseudo-inverse of Gᵀ, used by ObjectSE3Task
//                                 kinematic Jacobian (Stage B-4): J_obj =
//                                 (Gᵀ)⁺·J_c_stack.
//   P_N      ∈ R^{n_λ × n_λ}  — nullspace projector I - G_pinv·G; projects
//                                 a contact-wrench vector onto Null(G).
//   rank_G   ∈ [0, 6]         — numerical rank from LDLT diagonal pivots.
//
// Algorithm
// ─────────
// Always compute via the small 6×6 normal equations when n_λ ≥ 6:
//   M = G · Gᵀ                    (6 × 6, SPSD)
//   LDLT in-place on M
//   rank_G = # of diagonal pivots with |D_i| > tol_pivot
//   G_pivot regularization: damped right-inverse
//     G_pinv = Gᵀ · (M + ε·I)⁻¹   (RT-safe, no SVD)
//     (ε = max(tol_damp, machine_eps · trace(M)/6))
//   GT_pinv = ((M + ε·I)⁻¹) · G   (since (Gᵀ)⁺ = G · (G·Gᵀ)⁻¹ ; symmetric solve)
//   P_N = I_{n_λ} - G_pinv · G
//
// When n_λ < 6 (under-determined direction-wise), the dual normal equation
//   N = Gᵀ · G                    (n_λ × n_λ)
// is used instead for stability; same LDLT-based rank + damped solve.
//
// All allocations happen in Init() (worst-case sizing from
// ContactManagerConfig.max_contact_vars). Compute() is RT-safe: only block
// assignments, in-place LDLT on pre-allocated workspace, no heap traffic.
//
// Damping tolerance defaults match the order of magnitude used elsewhere
// in rtc_tsid (1e-9). YAML override via task config (Stage B-3+ tasks
// pass `tol_pivot` / `tol_damp` through their own configs if needed).
// ────────────────────────────────────────────────
class GraspCache {
 public:
  void Init(int max_contact_vars) noexcept;

  // Compute pinv/projector for the active-packed grasp matrix
  // G ∈ R^{6 × n_λ_active}. The active dim must be reported by the caller
  // (typically ContactManager::ActiveLambdaDim). RT-safe.
  void Compute(const Eigen::Ref<const Eigen::MatrixXd>& G, int n_lambda_active) noexcept;

  [[nodiscard]] int CurrentLambdaDim() const noexcept { return n_lambda_; }

  [[nodiscard]] int Rank() const noexcept { return rank_g_; }

  // Views into the cached buffers. Sized to (n_lambda_, 6) / (6, n_lambda_) /
  // (n_lambda_, n_lambda_). Block-views are returned by const-ref to
  // emphasize "do not capture beyond next Compute()".
  [[nodiscard]] Eigen::Block<const Eigen::MatrixXd> GPinv() const noexcept {
    return g_pinv_.topLeftCorner(n_lambda_, 6);
  }

  [[nodiscard]] Eigen::Block<const Eigen::MatrixXd> GTPinv() const noexcept {
    return gt_pinv_.topLeftCorner(6, n_lambda_);
  }

  [[nodiscard]] Eigen::Block<const Eigen::MatrixXd> ProjN() const noexcept {
    return p_n_.topLeftCorner(n_lambda_, n_lambda_);
  }

  // Damping / pivot tolerances (override before Compute() if needed).
  void SetPivotTolerance(double tol) noexcept { tol_pivot_ = tol; }

  void SetDampingTolerance(double tol) noexcept { tol_damp_ = tol; }

 private:
  int max_lambda_{0};
  int n_lambda_{0};
  int rank_g_{0};
  double tol_pivot_{1e-9};
  double tol_damp_{1e-9};

  // Output buffers (worst-case sized).
  Eigen::MatrixXd g_pinv_;   // [max × 6]
  Eigen::MatrixXd gt_pinv_;  // [6 × max]
  Eigen::MatrixXd p_n_;      // [max × max]

  // Workspace (pre-allocated in Init; Compute() only writes into these).
  Eigen::Matrix<double, 6, 6> m6_;     // G·Gᵀ + ε·I  (path A)
  Eigen::Matrix<double, 6, 6> minv6_;  // M⁻¹ via solveInPlace (path A)
  Eigen::MatrixXd nlam_;               // [max × max] Gᵀ·G + ε·I  (path B)
  Eigen::MatrixXd ninv_;               // [max × max] N⁻¹ via solveInPlace (path B)
  Eigen::MatrixXd minv_g_;  // [6 × max]   M⁻¹·G (path A) or G·N⁻¹ⱼ stash (path B)
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt_6_;
  Eigen::LDLT<Eigen::MatrixXd> ldlt_n_;
};

}  // namespace rtc::tsid
