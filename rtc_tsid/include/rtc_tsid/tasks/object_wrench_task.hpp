#pragma once

#include "rtc_tsid/contact/contact_manager.hpp"
#include "rtc_tsid/contact/grasp_cache.hpp"
#include "rtc_tsid/core/task_base.hpp"
#include "rtc_tsid/types/object_frame.hpp"

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Stage B-2: ObjectWrenchTask — drives the contact-resultant wrench at the
// object origin toward a desired wrench w_obj_des ∈ R^6 (force-3 + moment-3,
// world-aligned at p_o).
//
//   cost = w · ‖J · z - r‖²
//   J    = [0_{6×nv} | G]                 ∈ R^{6 × n_vars}
//   r    = w_obj_des                       ∈ R^6
//   z    = [a; λ_1; ... ; λ_K]            (n_vars = nv + max_contact_vars)
//
// Inactive contacts contribute 0 columns in G (same convention as ForceTask
// / ContactConstraint — active contacts get their cdim columns at the
// cumulative slot offset, inactive slots stay 0 so the QP solution drifts
// to 0 via Hessian regularization).
//
// ResidualDim() returns 6 when ≥ 1 contact is active, else 0. The one-tick
// lag (UpdateResidualDim called from ComputeResidual before formulation
// finalizes) matches ForceTask / ContactConsistencyTask.
//
// Caller wiring (DemoWbcController, Stage B-2 wiring out of scope):
//   1. SetContactManager(...)         — manager_cfg lookup
//   2. SetContactManagerRuntime(...)  — ContactManager for active G assembly
//   3. SetGraspCache(...)             — shared GraspCache (rank/projector
//                                       reuse with InternalForceTask in B-3)
//   4. SetObjectFrame(...)            — per-tick or per-phase object pose
//   5. SetDesiredWrench(...)          — RT-safe w_obj_des update (controller
//                                       or higher-level planner)
//
// The task itself does *not* run GraspCache::Compute() — that's the
// controller's responsibility once per tick so the cache is shared with
// other Stage B-3+ tasks. ObjectWrenchTask only reads ContactManager's
// active-packed G and unpacks it into the J_block's λ columns by
// re-walking ContactState (matches ForceTask::ComputeResidual logic).
// ────────────────────────────────────────────────
class ObjectWrenchTask final : public TaskBase {
 public:
  [[nodiscard]] std::string_view Name() const noexcept override { return "object_wrench"; }

  void Init(const pinocchio::Model& model, const RobotModelInfo& robot_info, PinocchioCache& cache,
            const YAML::Node& task_config) override;

  [[nodiscard]] int ResidualDim() const noexcept override { return current_residual_dim_; }

  void ComputeResidual(const PinocchioCache& cache, const ControlReference& ref,
                       const ContactState& contacts, int n_vars,
                       Eigen::Ref<Eigen::MatrixXd> J_block,
                       Eigen::Ref<Eigen::VectorXd> r_block) noexcept override;

  // Wire the manager config and resize the scratch buffer to
  // max_contact_vars. Call once at lifecycle configure (RT-unsafe due to
  // the workspace resize).
  void SetContactManager(const ContactManagerConfig* manager) noexcept;

  void SetContactManagerRuntime(const ContactManager* mgr_rt) noexcept { mgr_rt_ = mgr_rt; }

  void SetGraspCache(const GraspCache* cache) noexcept { grasp_cache_ = cache; }

  void SetObjectFrame(const ObjectFrame* object) noexcept { object_ = object; }

  // RT-safe: copies into local 6-vector buffer.
  void SetDesiredWrench(const Eigen::Matrix<double, 6, 1>& w_obj_des) noexcept {
    w_obj_des_ = w_obj_des;
  }

  void UpdateResidualDim(const ContactState& contacts) noexcept;

 private:
  int nv_{0};
  int current_residual_dim_{0};

  const ContactManagerConfig* manager_{nullptr};
  const ContactManager* mgr_rt_{nullptr};
  const GraspCache* grasp_cache_{nullptr};
  const ObjectFrame* object_{nullptr};

  Eigen::Matrix<double, 6, 1> w_obj_des_{Eigen::Matrix<double, 6, 1>::Zero()};

  // Scratch workspace: 6 × max_contact_vars holds the active-packed G
  // before unpacking into J_block λ columns. Pre-allocated at Init().
  Eigen::MatrixXd g_active_;
};

}  // namespace rtc::tsid
