#pragma once

#include <string>
#include <unordered_map>

#include "rtc_tsid/core/task_base.hpp"

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Contact force reference tracking task
//
// QP 변수 z = [a; λ_1; λ_2; ... ; λ_K] 에서
// 각 active contact의 λ_i에 대해:
//   cost = w * ‖λ_i - λ_des_i‖²
//
// J_block: λ_i 열에 Identity
// r_block: λ_des_i
//
// residual_dim = Σ(active contact_dim) — contact 상태에 따라 가변
// ────────────────────────────────────────────────
class ForceTask final : public TaskBase {
 public:
  [[nodiscard]] std::string_view name() const noexcept override {
    return "force";
  }

  void init(const pinocchio::Model& model,
            const RobotModelInfo& robot_info,
            PinocchioCache& cache,
            const YAML::Node& task_config) override;

  [[nodiscard]] int residual_dim() const noexcept override {
    return current_residual_dim_;
  }

  void compute_residual(
      const PinocchioCache& cache,
      const ControlReference& ref,
      const ContactState& contacts,
      int n_vars,
      Eigen::Ref<Eigen::MatrixXd> J_block,
      Eigen::Ref<Eigen::VectorXd> r_block) noexcept override;

  /// @brief ContactManagerConfig 포인터 설정 (init 후 외부에서 호출)
  void set_contact_manager(const ContactManagerConfig* manager) noexcept;

  /// @brief 개별 contact의 force reference 설정 (RT-safe)
  /// @param contact_index ContactManagerConfig 내 인덱스
  /// @param lambda_des 해당 contact의 desired force [contact_dim]
  void set_force_reference(int contact_index,
                           const Eigen::VectorXd& lambda_des) noexcept;

  /// @brief 전체 contact force reference 일괄 설정 (RT-safe)
  /// @param lambda_des_all 전체 contact force [max_contact_vars]
  void set_force_references(const Eigen::VectorXd& lambda_des_all) noexcept;

  /// @brief 현재 active contact 기준 residual_dim 갱신 (solve 전 호출)
  void update_residual_dim(const ContactState& contacts) noexcept;

 private:
  int nv_{0};
  int max_contact_vars_{0};
  int current_residual_dim_{0};
  const ContactManagerConfig* manager_{nullptr};

  // Per-contact force reference (max_contact_vars 크기로 pre-allocate)
  Eigen::VectorXd lambda_des_;   // [max_contact_vars]
  bool has_local_ref_{false};
};

}  // namespace rtc::tsid
