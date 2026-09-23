// ── q̇ᵘ: the unit-speed joint velocity behind v_dir,max (L3 §4.5, S6-B) ──────
//
// Damped least squares for unit speed along v̂ with the approach axis held:
//
//     J₅ = [ J_p (LOCAL_WORLD_ALIGNED, rows 0–2) ; J_ω (LOCAL, rows 3–4) ]
//     q̇ᵘ = J₅ᵀ (J₅J₅ᵀ + λ²I)⁻¹ [v̂; 0; 0]
//
// The SAME formula, rows and damping (λ = 1e-3) as the offline map's python
// (`rtc_tools.analysis.catch_speed_budget.dls_unit_velocity`,
// `DEFAULT_DLS_DAMPING`), which fed the S3.5b gate map. Before S6-B the runtime
// had no producer and the map's q̇ᵘ had nothing to be equivalent to (G3-I's
// "NOT_EVALUATED(S6.2)" half); this closes it.
//
// Fixed capacity (kMaxPlanNv columns), allocation-free after `Resize`,
// noexcept — the planner thread runs under RT-1~10.
#pragma once

#include "rtc_controllers/catching/trajectory.hpp"  // kMaxPlanNv
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <cmath>
#include <cstddef>
#include <span>

namespace rtc::catching {

/// The damping the map used (python `DEFAULT_DLS_DAMPING`).
inline constexpr double kUnitSpeedDamping = 1e-3;

struct UnitSpeedResult {
  Eigen::Vector3d jp_qdot_u{Eigen::Vector3d::Zero()};  ///< J_p q̇ᵘ — what q̇ᵘ achieves [m/s]
  bool valid{false};                                   ///< false: non-finite J or solve
};

class UnitSpeedSolver {
 public:
  /// Non-RT. Size the scratch for an `nv`-column model.
  void Resize(int nv) {
    nv_ = nv;
    j_lwa_.setZero(6, nv);
    j_local_.setZero(6, nv);
  }

  [[nodiscard]] int Nv() const noexcept { return nv_; }

  /// q̇ᵘ at `q` (model joint order, `Nv()` entries) into `qdot_u`. The handle
  /// must be this thread's own and carry no joint reorder (as for
  /// CatchPoseIk). RT-safe.
  [[nodiscard]] UnitSpeedResult Compute(rtc_urdf_bridge::RtModelHandle& model,
                                        pinocchio::FrameIndex frame, std::span<const double> q,
                                        const Eigen::Vector3d& v_hat, std::span<double> qdot_u,
                                        double damping = kUnitSpeedDamping) noexcept {
    UnitSpeedResult out;
    if (nv_ <= 0 || nv_ > static_cast<int>(kMaxPlanNv) ||
        q.size() < static_cast<std::size_t>(nv_) || qdot_u.size() < static_cast<std::size_t>(nv_) ||
        !v_hat.allFinite()) {
      return out;
    }
    model.ComputeJacobians(q.first(static_cast<std::size_t>(nv_)));
    model.GetFrameJacobian(frame, pinocchio::LOCAL_WORLD_ALIGNED, j_lwa_);
    model.GetFrameJacobian(frame, pinocchio::LOCAL, j_local_);

    j5_.setZero();
    for (int c = 0; c < nv_; ++c) {
      for (int r = 0; r < 3; ++r) {
        j5_(r, c) = j_lwa_(r, c);
      }
      j5_(3, c) = j_local_(3, c);
      j5_(4, c) = j_local_(4, c);
    }
    Eigen::Matrix<double, 5, 1> rhs;
    rhs << v_hat.x(), v_hat.y(), v_hat.z(), 0.0, 0.0;
    // Explicit loops rather than `j5 * j5ᵀ`: the inner dimension is dynamic,
    // and Eigen's GEMM path may allocate its blocking workspace (RT-1).
    Eigen::Matrix<double, 5, 5> a = Eigen::Matrix<double, 5, 5>::Zero();
    for (int r = 0; r < 5; ++r) {
      for (int s = 0; s < 5; ++s) {
        double acc = 0.0;
        for (int c = 0; c < nv_; ++c) {
          acc += j5_(r, c) * j5_(s, c);
        }
        a(r, s) = acc;
      }
    }
    a.diagonal().array() += damping * damping;
    const Eigen::LDLT<Eigen::Matrix<double, 5, 5>> ldlt(a);
    if (ldlt.info() != Eigen::Success) {
      return out;
    }
    const Eigen::Matrix<double, 5, 1> y = ldlt.solve(rhs);
    Eigen::Vector3d jp_qd = Eigen::Vector3d::Zero();
    for (int c = 0; c < nv_; ++c) {
      double qd = 0.0;
      for (int r = 0; r < 5; ++r) {
        qd += j5_(r, c) * y(r);
      }
      qdot_u[static_cast<std::size_t>(c)] = qd;
      jp_qd += qd * j5_.block<3, 1>(0, c);
    }
    out.jp_qdot_u = jp_qd;
    out.valid = jp_qd.allFinite() && y.allFinite();
    return out;
  }

 private:
  int nv_{0};
  Eigen::MatrixXd j_lwa_;    // 6 × nv, sized in Resize
  Eigen::MatrixXd j_local_;  // 6 × nv
  Eigen::Matrix<double, 5, static_cast<int>(kMaxPlanNv)> j5_{};
};

}  // namespace rtc::catching
