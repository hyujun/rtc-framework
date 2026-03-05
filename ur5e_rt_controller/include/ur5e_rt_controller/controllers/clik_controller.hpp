// ── Includes: project header first, then third-party, then C++ stdlib ──────────
#pragma once

#include "ur5e_rt_controller/rt_controller_interface.hpp"

// Suppress warnings emitted by Pinocchio / Eigen headers
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/jacobian.hpp>     // computeJointJacobians, getJointJacobian
#include <pinocchio/algorithm/kinematics.hpp>   // forwardKinematics (via computeJointJacobians)
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Cholesky>   // LDLT
#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <span>
#include <string>
#include <string_view>

namespace ur5e_rt_controller {

/// Closed-Loop Inverse Kinematics (CLIK) controller.
///
/// Controls the end-effector **position** (3-DOF) in Cartesian space.
/// Joint velocities are computed via the damped Jacobian pseudoinverse and
/// then integrated to produce absolute joint-position commands.  Surplus
/// DOF are exploited via a null-space secondary task that drives the robot
/// toward a preferred joint configuration, e.g. for joint-limit avoidance.
///
/// ### Control law
/// @code
///   pos_error    = x_des − FK(q)               [3D, metres]
///   J_pos        = J[0:3, :]                   [translational Jacobian, 3×nv]
///   J_pos^#      = J_pos^T (J_pos J_pos^T + λ²I)^{−1}   [damped pseudoinverse]
///   N            = I − J_pos^# J_pos            [null-space projector, nv×nv]
///
///   dq           = kp      * J_pos^# * pos_error          [primary task]
///              +   null_kp * N        * (q_null − q)       [secondary task]
///
///   q_cmd        = q + clamp(dq, ±v_max) * dt
/// @endcode
///
/// ### Target convention (`SetRobotTarget` / `/target_joint_positions` topic)
/// The 6 values sent to this controller are **NOT** joint angles:
///   - `target[0..2]` = desired TCP position  [x, y, z]  in world frame (m)
///   - `target[3..5]` = null-space reference joints 3–5 (rad);
///                      joints 0–2 use `kNullTarget` as reference
///
/// ### Usage — swap into custom_controller.cpp
/// @code
///   // 1. Include this header instead of pd_controller.hpp
///   // 2. Change controller_ member type to RTControllerInterface
///   // 3. Initialise:
///   //      controller_(std::make_unique<urtc::ClikController>(
///   //          "/opt/ros/humble/share/ur_description/urdf/ur5e.urdf",
///   //          urtc::ClikController::Gains{
///   //              .kp = 1.0, .damping = 0.01, .null_kp = 0.5}))
///   // 4. Remove the set_gains() call in DeclareAndLoadParameters()
/// @endcode
class ClikController final : public RTControllerInterface {
 public:
  // ── Gain / feature configuration ─────────────────────────────────────────
  struct Gains {
    double kp{1.0};                  ///< Cartesian position gain   [1/s]
    double damping{0.01};            ///< Damping factor λ for J^#  (singularity robustness)
    double null_kp{0.5};             ///< Null-space joint-centering gain [1/s]
    bool   enable_null_space{true};  ///< Enable null-space secondary task
  };

  /// @param urdf_path  Absolute path to the UR5e URDF file.
  /// @param gains      Gains and feature flags.
  /// @throws std::runtime_error  if the URDF cannot be parsed.
  explicit ClikController(std::string_view urdf_path, Gains gains = {});

  // ── RTControllerInterface — all methods are noexcept (RT safety) ──────────
  [[nodiscard]] ControllerOutput Compute(const ControllerState& state) noexcept override;

  void SetRobotTarget(std::span<const double> target) noexcept override;
  void SetHandTarget(std::span<const double> target)  noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override;

  void TriggerEstop()                            noexcept override;
  void ClearEstop()                              noexcept override;
  [[nodiscard]] bool IsEstopped() const          noexcept override;
  void SetHandEstop(bool active)                 noexcept override;

  // ── Accessors (non-RT reads only) ─────────────────────────────────────────
  void set_gains(const Gains& g) noexcept { gains_ = g; }
  [[nodiscard]] Gains gains()    const noexcept { return gains_; }

  /// Cached TCP position (world frame) from the most recent Compute().
  [[nodiscard]] std::array<double, 3> tcp_position() const noexcept {
    return tcp_position_;
  }

  /// Cached 3D Cartesian position error from the most recent Compute().
  [[nodiscard]] std::array<double, 3> position_error() const noexcept {
    return {pos_error_[0], pos_error_[1], pos_error_[2]};
  }

 private:
  // ── Pinocchio model + pre-allocated Data ─────────────────────────────────
  pinocchio::Model      model_;
  pinocchio::Data       data_;
  pinocchio::JointIndex end_id_{0};   ///< last joint index (end-effector)

  // ── Pre-allocated Eigen work buffers — zero heap alloc on the RT path ────
  Eigen::VectorXd q_;          ///< nv: joint positions
  Eigen::MatrixXd J_full_;     ///< 6×nv: full spatial Jacobian (LOCAL_WORLD_ALIGNED)
  Eigen::MatrixXd J_pos_;      ///< 3×nv: translational part of J_full_
  Eigen::Matrix3d JJt_;        ///< 3×3: J_pos * J_pos^T + λ²I
  Eigen::Matrix3d JJt_inv_;    ///< 3×3: (J_pos * J_pos^T + λ²I)^{-1}
  Eigen::MatrixXd Jpinv_;      ///< nv×3: damped pseudoinverse J_pos^#
  Eigen::MatrixXd N_;          ///< nv×nv: null-space projector I − J_pos^# J_pos
  Eigen::VectorXd dq_;         ///< nv: joint velocity command
  Eigen::VectorXd null_err_;   ///< nv: (q_null − q_current)
  Eigen::VectorXd null_dq_;    ///< nv: null-space contribution to dq
  Eigen::Vector3d pos_error_;  ///< 3: Cartesian position error

  // LDLT decomposition of JJt_ (3×3) — fixed-size → lives on the stack,
  // no dynamic allocation at construction or on the RT path.
  Eigen::LDLT<Eigen::Matrix3d> ldlt_;

  // ── Controller state ──────────────────────────────────────────────────────
  Gains gains_;
  std::array<double, 3>               tcp_target_{};
  /// Null-space reference configuration.  Joints 0–2 from this array;
  /// joints 3–5 are overwritten by SetRobotTarget(target[3..5]).
  std::array<double, kNumRobotJoints> null_target_{0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
  std::array<double, kNumHandJoints>  hand_target_{};
  std::array<double, 3>               tcp_position_{};  ///< diagnostic cache

  // ── E-STOP ────────────────────────────────────────────────────────────────
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};

  static constexpr std::array<double, kNumRobotJoints> kSafePosition{
      0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
  static constexpr double kMaxJointVelocity{2.0};

  // ── Helpers ───────────────────────────────────────────────────────────────
  [[nodiscard]] ControllerOutput ComputeEstop(const ControllerState& state) noexcept;

  [[nodiscard]] static std::array<double, kNumRobotJoints> ClampVelocity(
      std::array<double, kNumRobotJoints> dq) noexcept;
};

// ── Constructor ─────────────────────────────────────────────────────────────

inline ClikController::ClikController(std::string_view urdf_path, Gains gains)
    : data_(pinocchio::Model{}), gains_(gains) {
  pinocchio::urdf::buildModel(std::string(urdf_path), model_);
  data_   = pinocchio::Data(model_);
  end_id_ = static_cast<pinocchio::JointIndex>(model_.njoints - 1);

  // Pre-allocate all Eigen buffers to their final sizes.
  q_        = Eigen::VectorXd::Zero(model_.nv);
  J_full_   = Eigen::MatrixXd::Zero(6, model_.nv);
  J_pos_    = Eigen::MatrixXd::Zero(3, model_.nv);
  JJt_      = Eigen::Matrix3d::Zero();
  JJt_inv_  = Eigen::Matrix3d::Zero();
  Jpinv_    = Eigen::MatrixXd::Zero(model_.nv, 3);
  N_        = Eigen::MatrixXd::Identity(model_.nv, model_.nv);
  dq_       = Eigen::VectorXd::Zero(model_.nv);
  null_err_ = Eigen::VectorXd::Zero(model_.nv);
  null_dq_  = Eigen::VectorXd::Zero(model_.nv);
  pos_error_ = Eigen::Vector3d::Zero();
}

// ── RTControllerInterface implementation ────────────────────────────────────

inline ControllerOutput ClikController::Compute(
    const ControllerState& state) noexcept {
  if (estopped_) return ComputeEstop(state);

  // ── Step 1: copy joint state into Eigen vector ───────────────────────────
  for (Eigen::Index i = 0; i < model_.nv; ++i) {
    q_[i] = state.robot.positions[static_cast<std::size_t>(i)];
  }

  // ── Step 2: FK + Jacobians ───────────────────────────────────────────────
  // computeJointJacobians performs FK internally — data_.oMi is updated.
  pinocchio::computeJointJacobians(model_, data_, q_);
  pinocchio::getJointJacobian(model_, data_, end_id_,
                               pinocchio::LOCAL_WORLD_ALIGNED, J_full_);
  // Translational Jacobian: rows 0..2
  J_pos_.noalias() = J_full_.topRows(3);

  // ── Step 3: Cartesian position error ─────────────────────────────────────
  const Eigen::Vector3d tcp = data_.oMi[end_id_].translation();
  tcp_position_ = {tcp[0], tcp[1], tcp[2]};
  for (int i = 0; i < 3; ++i) {
    pos_error_[i] = tcp_target_[static_cast<std::size_t>(i)] - tcp[i];
  }

  // ── Step 4: Damped pseudoinverse  J_pos^# = J_pos^T (J_pos J_pos^T + λ²I)^{-1}
  // JJt_ is Eigen::Matrix3d (fixed-size) — LDLT is also fixed-size, no heap.
  JJt_.noalias() = J_pos_ * J_pos_.transpose();
  JJt_.diagonal().array() += gains_.damping * gains_.damping;
  ldlt_.compute(JJt_);
  // Solve (J_pos J_pos^T + λ²I) X = I₃  →  X = JJt_^{-1}  (fixed-size, no heap)
  JJt_inv_.noalias() = ldlt_.solve(Eigen::Matrix3d::Identity());
  // J_pos^# = J_pos^T * JJt_^{-1}   (nv×3)
  Jpinv_.noalias() = J_pos_.transpose() * JJt_inv_;

  // ── Step 5: Primary task  dq = kp * J_pos^# * pos_error ──────────────────
  dq_.noalias() = Jpinv_ * pos_error_;
  dq_ *= gains_.kp;

  // ── Step 6: Null-space secondary task ────────────────────────────────────
  // N = I − J_pos^# * J_pos  maps joint velocities into the null-space of
  // J_pos, leaving the primary Cartesian task unaffected.
  if (gains_.enable_null_space) {
    N_.setIdentity();
    // N_ -= Jpinv_ * J_pos_  →  in-place, noalias avoids temporaries
    N_.noalias() -= Jpinv_ * J_pos_;

    for (Eigen::Index i = 0; i < model_.nv; ++i) {
      null_err_[i] = null_target_[static_cast<std::size_t>(i)]
                   - state.robot.positions[static_cast<std::size_t>(i)];
    }
    null_dq_.noalias() = N_ * null_err_;
    null_dq_ *= gains_.null_kp;
    dq_ += null_dq_;
  }

  // ── Step 7: Clamp joint velocity and integrate ────────────────────────────
  // q_cmd = q + clamp(dq, ±v_max) * dt
  const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);
  std::array<double, kNumRobotJoints> dq_arr{};
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    dq_arr[i] = dq_[static_cast<Eigen::Index>(i)];
  }
  dq_arr = ClampVelocity(dq_arr);

  ControllerOutput output;
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    output.robot_commands[i] = state.robot.positions[i] + dq_arr[i] * dt;
  }
  return output;
}

inline void ClikController::SetRobotTarget(
    std::span<const double> target) noexcept {
  // First 3 values: Cartesian [x, y, z]
  const std::size_t n_cart = std::min(target.size(), std::size_t{3});
  for (std::size_t i = 0; i < n_cart; ++i) {
    tcp_target_[i] = target[i];
  }
  // Values 3..5: null-space reference for joints 3, 4, 5
  for (std::size_t i = 3; i < std::min(target.size(), kNumRobotJoints); ++i) {
    null_target_[i] = target[i];
  }
}

inline void ClikController::SetHandTarget(
    std::span<const double> target) noexcept {
  const std::size_t n = std::min(target.size(), kNumHandJoints);
  for (std::size_t i = 0; i < n; ++i) {
    hand_target_[i] = target[i];
  }
}

inline std::string_view ClikController::Name() const noexcept {
  return "ClikController";
}

inline void ClikController::TriggerEstop() noexcept {
  estopped_.store(true, std::memory_order_relaxed);
}

inline void ClikController::ClearEstop() noexcept {
  estopped_.store(false, std::memory_order_relaxed);
}

inline bool ClikController::IsEstopped() const noexcept {
  return estopped_.load(std::memory_order_relaxed);
}

inline void ClikController::SetHandEstop(bool active) noexcept {
  hand_estopped_.store(active, std::memory_order_relaxed);
}

// ── Private helpers ──────────────────────────────────────────────────────────

inline ControllerOutput ClikController::ComputeEstop(
    const ControllerState& state) noexcept {
  ControllerOutput output;
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    output.robot_commands[i] = state.robot.positions[i]
        + std::clamp(kSafePosition[i] - state.robot.positions[i],
                     -kMaxJointVelocity, kMaxJointVelocity)
        * ((state.dt > 0.0) ? state.dt : (1.0 / 500.0));
  }
  return output;
}

inline std::array<double, kNumRobotJoints> ClikController::ClampVelocity(
    std::array<double, kNumRobotJoints> dq) noexcept {
  for (auto& v : dq) {
    v = std::clamp(v, -kMaxJointVelocity, kMaxJointVelocity);
  }
  return dq;
}

}  // namespace ur5e_rt_controller
