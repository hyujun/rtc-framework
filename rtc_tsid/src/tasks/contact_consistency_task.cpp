#include "rtc_tsid/tasks/contact_consistency_task.hpp"

#include <cmath>

namespace rtc::tsid {

void ContactConsistencyTask::Init(const pinocchio::Model& /*model*/,
                                  const RobotModelInfo& robot_info, PinocchioCache& /*cache*/,
                                  const YAML::Node& task_config) {
  nv_ = robot_info.nv;

  if (task_config && task_config["weight"]) {
    weight_ = task_config["weight"].as<double>();
  }
  if (task_config && task_config["priority"]) {
    priority_ = task_config["priority"].as<int>();
  }

  current_residual_dim_ = 0;
}

void ContactConsistencyTask::SetContactManager(const ContactManagerConfig* manager) noexcept {
  manager_ = manager;
}

void ContactConsistencyTask::ComputeResidual(const PinocchioCache& cache,
                                             const ControlReference& /*ref*/,
                                             const ContactState& contacts, int /*n_vars*/,
                                             Eigen::Ref<Eigen::MatrixXd> J_block,
                                             Eigen::Ref<Eigen::VectorXd> r_block) noexcept {
  // ResidualDim() 은 WQPFormulation 이 미리 호출하여 J_block / r_block 의 row 수를
  // 결정한다 (ForceTask 와 같은 한-tick 지연 패턴). 본 ComputeResidual 호출 *이후*
  // current_residual_dim_ 가 갱신되므로, 첫 tick 또는 active contact 가 바뀐 직후
  // 한 tick 은 contribution 이 비어 있다 — 그래도 무해 (cost 0).
  current_residual_dim_ = contacts.active_contact_vars;
  if (!manager_ || current_residual_dim_ == 0) {
    return;
  }

  // J_block / r_block 은 호출자에 의해 zero-init 되어 있음.
  // active contact 순으로 row 를 채운다.
  int row = 0;
  for (size_t i = 0; i < contacts.contacts.size(); ++i) {
    if (!contacts.contacts[i].active) {
      continue;
    }
    if (i >= cache.contact_frames.size() || i >= manager_->contacts.size()) {
      continue;
    }
    const int cdim = manager_->contacts[i].contact_dim;
    const auto& Jc = cache.contact_frames[i].J;     // [6 × nv] LOCAL_WORLD_ALIGNED
    const auto& dJv = cache.contact_frames[i].dJv;  // [6]

    // Stage A-5b: sqrt(s_i) cost scaling. Cost = w·‖J_c·a + dotJ_c·v‖²
    // becomes w·s_i·‖...‖² by multiplying both J and r blocks by √s_i.
    // s_i=1 (default) is byte-identical to pre-A-5b behaviour.
    const double sqrt_s = std::sqrt(contacts.contacts[i].activation);

    // J_block[row:row+cdim, 0:nv] = √s · J_c[:cdim, :]
    J_block.block(row, 0, cdim, nv_) = sqrt_s * Jc.topRows(cdim);
    // J_block λ-columns 는 zero (no coupling).
    // r_block[row:row+cdim] = √s · (-dotJ_c · v)
    r_block.segment(row, cdim) = -sqrt_s * dJv.head(cdim);

    row += cdim;
  }
}

}  // namespace rtc::tsid
