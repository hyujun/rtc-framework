#pragma once

#include <string>

#include "rtc_tsid/core/task_base.hpp"

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Centroidal momentum regularization / tracking task
//
// Mode (a) angular_regularize (default):
//   Angular momentum → 0 regularization
//   J_block = Ag[3:6, :]  (angular 부분)  [3 × nv]
//   r_block = -hg_drift[3:6]
//   residual_dim = 3
//
// Mode (b) full_track:
//   Full centroidal momentum reference tracking
//   J_block = Ag  [6 × nv]
//   r_block = hg_dot_des - hg_drift
//   residual_dim = 6
// ────────────────────────────────────────────────
class MomentumTask final : public TaskBase {
 public:
  enum class Mode {
    kAngularRegularize,  // angular momentum → 0
    kFullTrack           // full 6D momentum reference tracking
  };

  [[nodiscard]] std::string_view name() const noexcept override {
    return "momentum";
  }

  void init(const pinocchio::Model& model,
            const RobotModelInfo& robot_info,
            PinocchioCache& cache,
            const YAML::Node& task_config) override;

  [[nodiscard]] int residual_dim() const noexcept override {
    return (mode_ == Mode::kAngularRegularize) ? 3 : 6;
  }

  void compute_residual(
      const PinocchioCache& cache,
      const ControlReference& ref,
      const ContactState& contacts,
      int n_vars,
      Eigen::Ref<Eigen::MatrixXd> J_block,
      Eigen::Ref<Eigen::VectorXd> r_block) noexcept override;

  /// @brief Full-track 모드용 momentum rate reference 설정 (RT-safe)
  /// @param hg_dot_des Desired centroidal momentum rate [6]
  void set_momentum_reference(
      const Eigen::Matrix<double, 6, 1>& hg_dot_des) noexcept;

 private:
  int nv_{0};
  Mode mode_{Mode::kAngularRegularize};

  // Full-track reference
  Eigen::Matrix<double, 6, 1> hg_dot_des_;
};

}  // namespace rtc::tsid
