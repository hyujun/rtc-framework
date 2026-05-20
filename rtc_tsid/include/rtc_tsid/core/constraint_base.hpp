#pragma once

#include "rtc_tsid/types/wbc_types.hpp"

#include <Eigen/Core>

#include <string_view>

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Constraint 추상 인터페이스
//
// Equality:   A·z = b
// Inequality: l ≤ C·z ≤ u
// WQP/HQP 양쪽에서 동일하게 동작 (모든 level에 공통 적용)
// ────────────────────────────────────────────────
class ConstraintBase {
 public:
  virtual ~ConstraintBase() = default;

  [[nodiscard]] virtual std::string_view Name() const noexcept = 0;

  virtual void Init(const pinocchio::Model& model, const RobotModelInfo& robot_info,
                    PinocchioCache& cache, const YAML::Node& constraint_config) = 0;

  // Constraint 차원 (ContactState에 따라 매 tick 달라질 수 있음)
  [[nodiscard]] virtual int EqDim(const ContactState& contacts) const noexcept = 0;
  [[nodiscard]] virtual int IneqDim(const ContactState& contacts) const noexcept = 0;

  // Equality: A·z = b — RT-safe
  // A_block: [eq_dim × n_vars], caller가 zero-init
  virtual void ComputeEquality(const PinocchioCache& cache, const ContactState& contacts,
                                const RobotModelInfo& robot_info, int n_vars,
                                Eigen::Ref<Eigen::MatrixXd> A_block,
                                Eigen::Ref<Eigen::VectorXd> b_block) noexcept = 0;

  // Inequality: l ≤ C·z ≤ u — RT-safe
  // C_block: [ineq_dim × n_vars], caller가 zero-init
  virtual void ComputeInequality(const PinocchioCache& cache, const ContactState& contacts,
                                  const RobotModelInfo& robot_info, int n_vars,
                                  Eigen::Ref<Eigen::MatrixXd> C_block,
                                  Eigen::Ref<Eigen::VectorXd> l_block,
                                  Eigen::Ref<Eigen::VectorXd> u_block) noexcept = 0;

  void SetActive(bool active) noexcept { active_ = active; }

  [[nodiscard]] bool IsActive() const noexcept { return active_; }

 protected:
  bool active_{true};
};

}  // namespace rtc::tsid
