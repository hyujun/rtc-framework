#pragma once

#include "rtc_tsid/types/wbc_types.hpp"

#include <Eigen/Core>

#include <string_view>

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Task 추상 인터페이스
//
// 각 task는 (J, r, w) 를 제공: cost = w * ‖J·z - r‖²
// WQP: weight만 사용, priority 무시
// HQP: priority로 level 분류, 동일 level 내 weight로 soft priority
// ComputeResidual()은 WQP/HQP 양쪽에서 동일하게 호출됨
// ────────────────────────────────────────────────
class TaskBase {
 public:
  virtual ~TaskBase() = default;

  [[nodiscard]] virtual std::string_view Name() const noexcept = 0;

  // 초기화: buffer pre-allocate, required frame 등록
  // PinocchioCache는 non-const (RegisterFrame 호출 가능)
  virtual void Init(const pinocchio::Model& model, const RobotModelInfo& robot_info,
                    PinocchioCache& cache, const YAML::Node& task_config) = 0;

  // QP cost에 기여하는 residual 차원
  [[nodiscard]] virtual int ResidualDim() const noexcept = 0;

  // (J, r) 계산 — RT-safe: 내부 할당 금지
  // J_block: [ResidualDim × n_vars], caller가 zero-init
  // r_block: [ResidualDim]
  virtual void ComputeResidual(const PinocchioCache& cache, const ControlReference& ref,
                               const ContactState& contacts, int n_vars,
                               Eigen::Ref<Eigen::MatrixXd> J_block,
                               Eigen::Ref<Eigen::VectorXd> r_block) noexcept = 0;

  // Weight (WQP 사용, HQP 동일 level 내 상대 비중)
  void SetWeight(double w) noexcept { weight_ = w; }

  [[nodiscard]] double Weight() const noexcept { return weight_; }

  // Priority level (HQP level 분류용, WQP에서 무시)
  // 0 = 최고 priority
  void SetPriority(int level) noexcept { priority_ = level; }

  [[nodiscard]] int Priority() const noexcept { return priority_; }

  // 활성/비활성
  void SetActive(bool active) noexcept { active_ = active; }

  [[nodiscard]] bool IsActive() const noexcept { return active_; }

 protected:
  double weight_{1.0};
  int priority_{0};
  bool active_{true};
};

}  // namespace rtc::tsid
