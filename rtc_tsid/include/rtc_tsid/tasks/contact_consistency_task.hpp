#pragma once

#include "rtc_tsid/core/task_base.hpp"

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Contact acceleration consistency task (soft)
//
// Rigid no-slip contact at the acceleration level:
//    J_c · a + dotJ_c · v = 0
//
// As a soft TSID cost (rather than a hard equality, which couples poorly with
// contact activation ramping):
//    cost = w · ‖J_c · a + dotJ_c · v‖²
//
// In QP form J_block · z = r_block:
//    J_block = [J_c_stacked   0_{n_λ × n_λ}]    (active contacts only)
//    r_block = -dotJ_c · v                       (move bias to RHS)
//
// residual_dim = Σ(active contact_dim) — gated on ContactManagerConfig and
// the runtime ContactState. Reuses PinocchioCache::contact_frames cache.
//
// YAML keys:
//    weight   : double (default 1.0)
//    priority : int (default 0)
//
// Note on hard mode: the guide §3 allows a YAML `hard_constraint: true` flag
// to register a ContactConstraint instead. That dispatch lives in the
// controller (BuildTsidTasks/Constraints), not here — this class is the
// soft task path only, by design.
// ────────────────────────────────────────────────
class ContactConsistencyTask final : public TaskBase {
 public:
  [[nodiscard]] std::string_view Name() const noexcept override { return "contact_consistency"; }

  void Init(const pinocchio::Model& model, const RobotModelInfo& robot_info, PinocchioCache& cache,
            const YAML::Node& task_config) override;

  [[nodiscard]] int ResidualDim() const noexcept override { return current_residual_dim_; }

  void ComputeResidual(const PinocchioCache& cache, const ControlReference& ref,
                       const ContactState& contacts, int n_vars,
                       Eigen::Ref<Eigen::MatrixXd> J_block,
                       Eigen::Ref<Eigen::VectorXd> r_block) noexcept override;

  // ContactManagerConfig 포인터 설정 (init 후 controller 가 wire). manager 없이는
  // residual 이 0 contribution (no-op) — 안전 fallback.
  void SetContactManager(const ContactManagerConfig* manager) noexcept;

 private:
  int nv_{0};
  int current_residual_dim_{0};
  const ContactManagerConfig* manager_{nullptr};
};

}  // namespace rtc::tsid
