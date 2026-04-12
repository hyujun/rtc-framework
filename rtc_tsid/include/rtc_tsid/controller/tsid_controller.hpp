#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>

#include "rtc_tsid/core/controller_base.hpp"
#include "rtc_tsid/core/formulation_base.hpp"
#include "rtc_tsid/types/qp_types.hpp"

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Top-level TSID Controller
//
// PinocchioCache + FormulationBase(WQP/HQP) + τ 역산
// TSIDController는 formulation 종류를 모름 (Strategy Pattern)
// ────────────────────────────────────────────────
class TSIDController final : public ControllerBase {
 public:
  void init(const pinocchio::Model& model,
            const RobotModelInfo& robot_info,
            const YAML::Node& config) override;

  [[nodiscard]] CommandOutput compute(
      const ControlState& state,
      const ControlReference& ref,
      const PinocchioCache& cache,
      const ContactState& contacts) noexcept override;

  void reset() noexcept override;

  [[nodiscard]] std::string_view name() const noexcept override {
    return "tsid";
  }

  // Phase preset 적용
  void apply_phase_preset(const std::string& preset_name) noexcept;
  void apply_phase_preset(const PhasePreset& preset) noexcept;

  // Contact 활성/비활성 (ContactState 외부에서 직접 제어 시)
  void activate_contact(int idx) noexcept;
  void deactivate_contact(int idx) noexcept;

  // 연구용: formulation 직접 접근
  [[nodiscard]] FormulationBase& formulation() { return *formulation_; }
  [[nodiscard]] const FormulationBase& formulation() const {
    return *formulation_;
  }

 private:
  RobotModelInfo robot_info_;
  std::unique_ptr<FormulationBase> formulation_;

  // Phase presets (YAML에서 로드)
  std::unordered_map<std::string, PhasePreset> presets_;

  // τ 역산용 pre-allocated buffers
  Eigen::VectorXd tau_full_;      // [nv]
  Eigen::VectorXd tau_actuated_;  // [n_actuated]
  Eigen::VectorXd JcT_lambda_;   // [nv]

  CommandOutput output_;
};

}  // namespace rtc::tsid
