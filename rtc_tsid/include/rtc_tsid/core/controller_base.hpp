#pragma once

#include "rtc_tsid/types/wbc_types.hpp"

#include <string_view>

namespace rtc::tsid {

// ────────────────────────────────────────────────
// TSID 내부 Controller 인터페이스
// (RTControllerInterface와는 별도 — ROS2 통합은 Phase 3)
// ────────────────────────────────────────────────
class ControllerBase {
 public:
  virtual ~ControllerBase() = default;

  // 초기화 (동적 할당 허용)
  virtual void Init(const pinocchio::Model& model, const RobotModelInfo& robot_info,
                    const YAML::Node& config) = 0;

  // 매 tick 호출 (RT-safe). Returns the controller's own output buffer, valid
  // until the next Compute / Reset — a by-value return would copy its Eigen
  // vectors, i.e. allocate, on every tick.
  [[nodiscard]] virtual const CommandOutput& Compute(const ControlState& state,
                                                     const ControlReference& ref,
                                                     const PinocchioCache& cache,
                                                     const ContactState& contacts) noexcept = 0;

  virtual void Reset() noexcept = 0;

  [[nodiscard]] virtual std::string_view Name() const noexcept = 0;
};

}  // namespace rtc::tsid
