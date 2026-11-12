#include "rtc_tsid/controller/tsid_controller.hpp"

#include "rtc_tsid/formulation/formulation_factory.hpp"

#include <algorithm>

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

const CommandOutput& TSIDController::SkipTick() noexcept {
  output_.ResetDiagnostics();  // qp_converged / tau_valid false, qp_status −1
  return output_;
}

const CommandOutput& TSIDController::Compute(const ControlState& /*state*/,
                                             const ControlReference& ref,
                                             const PinocchioCache& cache,
                                             const ContactState& contacts) noexcept {
  // 1. Formulation solve (WQP 또는 HQP)
  const auto& result = formulation_->Solve(cache, ref, contacts, robot_info_);

  output_.qp_converged = result.converged;
  output_.solve_time_us = result.solve_time_us;
  output_.solve_levels = result.levels_solved;
  // Filled before the early return: the failure classification is exactly
  // what the non-converged path has to leave behind (#551).
  output_.qp_status = result.status;
  output_.qp_iterations = result.iterations;
  output_.qp_non_finite = result.non_finite;
  output_.n_eq_used = result.n_eq_used;
  output_.n_ineq_used = result.n_ineq_used;
  output_.rows_dropped = result.rows_dropped;
  output_.outer_iterations = result.outer_iterations;
  // Until the torque below is known to be finite, this tick's outputs are the
  // last valid ones (WD-60). An HQP's solution is that of the last level it
  // solved, so a lower level failing still leaves a usable torque.
  output_.tau_valid = false;

  if (!result.converged) {
    return output_;
  }

  const int nv = robot_info_.nv;
  const int na = robot_info_.n_actuated;

  // 2. a and the full λ block live in the solver's x_opt (stable for this
  // call): [a (nv); λ (max_contact_vars)]. Inactive contacts' λ slots are ≈ 0
  // by Hessian regularization (no task/constraint references them).
  const int n_lambda_full = contact_cfg_.max_contact_vars;
  const auto a = result.x_opt.head(nv);

  // 3. τ 역산: tau = S · (M·a + h - Jcᵀ·λ)
  tau_full_.noalias() = cache.M * a;
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
          Jc.topRows(cdim).transpose() * result.x_opt.segment(nv + lambda_offset, cdim);

      lambda_offset += cdim;
    }

    tau_full_ -= JcT_lambda_;
  }

  // S projection (noalias: the product would otherwise go through a heap
  // temporary on the RT tick)
  tau_actuated_.noalias() = robot_info_.S * tau_full_;
  if (!tau_actuated_.allFinite() || !result.x_opt.head(nv + n_lambda_full).allFinite()) {
    // A converged solve whose torque overflowed: reported like any other
    // non-finite tick, so a consumer gating on qp_converged sees it too.
    output_.qp_converged = false;
    output_.qp_non_finite = true;
    return output_;
  }

  // 4. Publish the valid tick and tell the constraints what was applied.
  output_.a_opt.head(nv) = a;
  if (n_lambda_full > 0) {
    output_.lambda_opt.head(n_lambda_full) = result.x_opt.segment(nv, n_lambda_full);
    // A slot not in contact carries no force: its λ enters no row but the
    // 1e-8 regularisation, so the solver leaves it where the warm start put
    // it. Report zero there, as τ above already assumes.
    int offset = 0;
    for (size_t i = 0; i < contacts.contacts.size() && offset < n_lambda_full; ++i) {
      const int cdim =
          (i < contact_cfg_.contacts.size()) ? contact_cfg_.contacts[i].contact_dim : 3;
      if (!contacts.contacts[i].active || i >= cache.contact_frames.size()) {
        output_.lambda_opt.segment(offset, std::min(cdim, n_lambda_full - offset)).setZero();
      }
      offset += cdim;
    }
  }
  output_.tau.head(na) = tau_actuated_;
  output_.tau_valid = true;
  formulation_->OnTorqueApplied(output_.tau);

  return output_;
}

void TSIDController::Reset() noexcept {
  if (formulation_) {
    formulation_->Reset();
  }
  output_.ResetDiagnostics();
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
