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

  /// @brief Add this task's weighted cost straight into the WQP cost, without
  /// the dense (J, r) round trip: H += w·JᵀJ, g −= w·Jᵀr.
  ///
  /// Optional. The default returns false and touches nothing — the caller then
  /// takes the dense path (ComputeResidual + the two products). A task whose J
  /// has at most one non-zero entry per column (a selection of QP variables,
  /// possibly scaled) can override it and return true.
  ///
  /// Contract of an override:
  ///  - the H and g it leaves are the ones the dense path would have left, as
  ///    the same doubles (on a build that does not fuse a multiply into an
  ///    add; with FMA the last bit may differ): per touched column c with the entry d and the
  ///    residual r, `H(c,c) += w * (d * d)` and `g(c) -= w * (d * r)` — the
  ///    weight multiplies the finished product, as the dense product does (a
  ///    `(w * d) * d` differs in the last bit for d ≠ 1). Other entries are
  ///    left alone;
  ///  - it has every side effect and early return of ComputeResidual (a
  ///    residual dimension it updates, a reference it ignores);
  ///  - RT-safe: no allocation, no throw.
  /// A non-finite r, w or d must leave a non-finite entry in g (the formulas
  /// above do: the three are multiplied together). The dense product spreads
  /// it over all of g (0 × NaN) and the solve is refused; the caller spreads
  /// what an override leaves in one column, to the same refusal — left there,
  /// the solver can return its previous iterate as solved.
  ///
  /// @param n_vars  QP variable count (H is n_vars × n_vars, g is n_vars)
  /// @param w       the task weight the caller read (Weight())
  /// @return true when the cost was added, false when the caller must take the
  ///         dense path
  [[nodiscard]] virtual bool AccumulateCost(const PinocchioCache& /*cache*/,
                                            const ControlReference& /*ref*/,
                                            const ContactState& /*contacts*/, int /*n_vars*/,
                                            double /*w*/, Eigen::Ref<Eigen::MatrixXd> /*H*/,
                                            Eigen::Ref<Eigen::VectorXd> /*g*/) noexcept {
    return false;
  }

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
