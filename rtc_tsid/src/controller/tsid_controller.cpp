#include "rtc_tsid/controller/tsid_controller.hpp"

#include "rtc_tsid/formulation/formulation_factory.hpp"

namespace rtc::tsid {

void TSIDController::Init(const pinocchio::Model& model, const RobotModelInfo& robot_info,
                          const YAML::Node& config) {
  robot_info_ = robot_info;

  // ContactManagerConfig 보관 — τ 역산에서 active contact 의 contact_dim 조회.
  contact_cfg_ = ContactManagerConfig{};
  if (config) {
    contact_cfg_.Load(config, model);
  }

  // Formulation 생성 (WQP or HQP)
  formulation_ = CreateFormulation(model, robot_info, contact_cfg_, config);

  // Phase presets 로드
  if (config) {
    presets_ = LoadPhasePresets(config);
  }

  // τ 역산 버퍼
  tau_full_.setZero(robot_info.nv);
  tau_actuated_.setZero(robot_info.n_actuated);
  JcT_lambda_.setZero(robot_info.nv);

  output_.Init(robot_info.nv, robot_info.n_actuated, contact_cfg_.max_contact_vars);
}

CommandOutput TSIDController::Compute(const ControlState& /*state*/, const ControlReference& ref,
                                      const PinocchioCache& cache,
                                      const ContactState& contacts) noexcept {
  // 1. Formulation solve (WQP 또는 HQP)
  const auto& result = formulation_->Solve(cache, ref, contacts, robot_info_);

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

  // Stage A-5a: fixed-dim QP — extract the full λ block (size =
  // max_contact_vars). Inactive contacts' λ slots are ≈ 0 by Hessian
  // regularization (no task/constraint references them).
  const int n_lambda_full = contact_cfg_.max_contact_vars;
  if (n_lambda_full > 0) {
    output_.lambda_opt.head(n_lambda_full) = result.x_opt.segment(nv, n_lambda_full);
  }

  // 3. τ 역산: tau = S · (M·a + h - Jcᵀ·λ)
  tau_full_.noalias() = cache.M * output_.a_opt.head(nv);
  tau_full_ += cache.h;

  // Jcᵀ·λ 조립 — fixed-dim λ index per contact slot. Skip inactive contacts
  // (their λ ≈ 0 contributes nothing) but advance offset so active contacts
  // pick up the right segment.
  if (n_lambda_full > 0) {
    JcT_lambda_.setZero();
    int lambda_offset = 0;

    for (size_t i = 0; i < contacts.contacts.size(); ++i) {
      const int cdim =
          (i < contact_cfg_.contacts.size()) ? contact_cfg_.contacts[i].contact_dim : 3;

      if (!contacts.contacts[i].active || i >= cache.contact_frames.size()) {
        lambda_offset += cdim;
        continue;
      }

      if (lambda_offset + cdim > n_lambda_full)
        break;

      const auto& Jc = cache.contact_frames[i].J;
      // Jcᵀ·λ_i = Jc[:cdim, :]ᵀ · λ_i
      JcT_lambda_.noalias() +=
          Jc.topRows(cdim).transpose() * output_.lambda_opt.segment(lambda_offset, cdim);

      lambda_offset += cdim;
    }

    tau_full_ -= JcT_lambda_;
  }

  // S projection
  output_.tau.head(na) = robot_info_.S * tau_full_;

  return output_;
}

void TSIDController::Reset() noexcept {
  output_.qp_converged = false;
  output_.solve_time_us = 0.0;
  output_.solve_levels = 0;
  output_.tau.setZero();
  output_.a_opt.setZero();
  output_.lambda_opt.setZero();
}

void TSIDController::ApplyPhasePreset(const std::string& preset_name) noexcept {
  auto it = presets_.find(preset_name);
  if (it != presets_.end()) {
    formulation_->ApplyPreset(it->second);
  }
}

void TSIDController::ApplyPhasePreset(const PhasePreset& preset) noexcept {
  formulation_->ApplyPreset(preset);
}

void TSIDController::ActivateContact(int /*idx*/) noexcept {
  // ContactState는 외부에서 관리 — 여기서는 no-op
  // 필요 시 내부 ContactState를 갖고 관리 가능
}

void TSIDController::DeactivateContact(int /*idx*/) noexcept {}

}  // namespace rtc::tsid
