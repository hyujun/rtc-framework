#include "rtc_tsid/controller/tsid_controller.hpp"

#include "rtc_tsid/formulation/formulation_factory.hpp"

namespace rtc::tsid {

void TSIDController::init(const pinocchio::Model& model,
                          const RobotModelInfo& robot_info,
                          const YAML::Node& config) {
  robot_info_ = robot_info;

  // ContactManagerConfig (TSID controller가 자체 관리하지 않음 — 외부 주입)
  ContactManagerConfig contact_cfg;
  if (config) {
    contact_cfg.load(config, model);
  }

  // Formulation 생성 (WQP or HQP)
  formulation_ = create_formulation(model, robot_info, contact_cfg, config);

  // Phase presets 로드
  if (config) {
    presets_ = load_phase_presets(config);
  }

  // τ 역산 버퍼
  tau_full_.setZero(robot_info.nv);
  tau_actuated_.setZero(robot_info.n_actuated);
  JcT_lambda_.setZero(robot_info.nv);

  output_.init(robot_info.nv, robot_info.n_actuated,
               contact_cfg.max_contact_vars);
}

CommandOutput TSIDController::compute(
    const ControlState& /*state*/,
    const ControlReference& ref,
    const PinocchioCache& cache,
    const ContactState& contacts) noexcept {
  // 1. Formulation solve (WQP 또는 HQP)
  const auto& result =
      formulation_->solve(cache, ref, contacts, robot_info_);

  output_.qp_converged = result.converged;
  output_.solve_time_us = result.solve_time_us;
  output_.solve_levels = result.levels_solved;

  if (!result.converged) {
    return output_;
  }

  const int nv = robot_info_.nv;
  const int na = robot_info_.n_actuated;

  // 2. Extract a_opt, lambda_opt
  output_.a_opt.head(nv) = result.x_opt.head(nv);

  const int n_lambda = contacts.active_contact_vars;
  if (n_lambda > 0) {
    output_.lambda_opt.head(n_lambda) = result.x_opt.segment(nv, n_lambda);
  }

  // 3. τ 역산: tau = S · (M·a + h - Jcᵀ·λ)
  tau_full_.noalias() = cache.M * output_.a_opt.head(nv);
  tau_full_ += cache.h;

  // Jcᵀ·λ 조립
  if (n_lambda > 0) {
    JcT_lambda_.setZero();
    int lambda_offset = 0;

    for (size_t i = 0; i < contacts.contacts.size(); ++i) {
      if (!contacts.contacts[i].active) continue;
      if (i >= cache.contact_frames.size()) continue;

      // Contact dim 추론: lambda 남은 크기에서 판단
      // 보수적으로 3 사용 (point contact)
      const int cdim = 3;  // TODO: ContactManagerConfig 참조

      if (lambda_offset + cdim > n_lambda) break;

      const auto& Jc = cache.contact_frames[i].J;
      // Jcᵀ·λ_i = Jc[:cdim, :]ᵀ · λ_i
      JcT_lambda_.noalias() +=
          Jc.topRows(cdim).transpose() *
          output_.lambda_opt.segment(lambda_offset, cdim);

      lambda_offset += cdim;
    }

    tau_full_ -= JcT_lambda_;
  }

  // S projection
  output_.tau.head(na) = robot_info_.S * tau_full_;

  return output_;
}

void TSIDController::reset() noexcept {
  output_.qp_converged = false;
  output_.solve_time_us = 0.0;
  output_.solve_levels = 0;
  output_.tau.setZero();
  output_.a_opt.setZero();
  output_.lambda_opt.setZero();
}

void TSIDController::apply_phase_preset(
    const std::string& preset_name) noexcept {
  auto it = presets_.find(preset_name);
  if (it != presets_.end()) {
    formulation_->apply_preset(it->second);
  }
}

void TSIDController::apply_phase_preset(
    const PhasePreset& preset) noexcept {
  formulation_->apply_preset(preset);
}

void TSIDController::activate_contact(int /*idx*/) noexcept {
  // ContactState는 외부에서 관리 — 여기서는 no-op
  // 필요 시 내부 ContactState를 갖고 관리 가능
}

void TSIDController::deactivate_contact(int /*idx*/) noexcept {
}

}  // namespace rtc::tsid
