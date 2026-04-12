#pragma once

#include <memory>
#include <string_view>

#include "rtc_tsid/core/constraint_base.hpp"
#include "rtc_tsid/core/task_base.hpp"
#include "rtc_tsid/types/qp_types.hpp"
#include "rtc_tsid/types/wbc_types.hpp"

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Formulation Strategy 추상 인터페이스
//
// WQPFormulation: 모든 task weighted sum → 단일 QP
// HQPFormulation: level별 순차 QP + cascaded equality
//
// TSIDController는 FormulationBase*만 보유 → WQP/HQP 무관
// ────────────────────────────────────────────────
class FormulationBase {
 public:
  virtual ~FormulationBase() = default;

  // 초기화 (동적 할당 허용)
  virtual void init(const pinocchio::Model& model,
                    const RobotModelInfo& robot_info,
                    const ContactManagerConfig& contact_cfg,
                    const YAML::Node& config) = 0;

  // Task/Constraint 등록 (init 단계에서만)
  virtual void add_task(std::unique_ptr<TaskBase> task) = 0;
  virtual void add_constraint(std::unique_ptr<ConstraintBase> constraint) = 0;

  // 이름으로 조회
  [[nodiscard]] virtual TaskBase* get_task(std::string_view name) = 0;
  [[nodiscard]] virtual ConstraintBase* get_constraint(
      std::string_view name) = 0;

  // Phase preset 적용 (task active/weight/priority + constraint active)
  virtual void apply_preset(const PhasePreset& preset) noexcept = 0;

  // 매 tick: (J, r) 수집 + QP 조립 + solve → 최적해 반환
  // WQP: 단일 QP solve, HQP: level별 순차 solve
  // ⚠ RT-safe: zero-alloc
  [[nodiscard]] virtual const SolveResult& solve(
      const PinocchioCache& cache,
      const ControlReference& ref,
      const ContactState& contacts,
      const RobotModelInfo& robot_info) noexcept = 0;

  // Formulation 종류 식별
  [[nodiscard]] virtual std::string_view type() const noexcept = 0;
};

}  // namespace rtc::tsid
