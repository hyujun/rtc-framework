#pragma once

#include "rtc_tsid/contact/contact_manager.hpp"
#include "rtc_tsid/contact/grasp_cache.hpp"
#include "rtc_tsid/contact/object_state_provider.hpp"
#include "rtc_tsid/core/task_base.hpp"
#include "rtc_tsid/types/object_frame.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/se3.hpp>
#pragma GCC diagnostic pop

#include <array>

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Stage B-4: ObjectSE3Task — drives the *object* SE(3) pose toward a desired
// placement through the kinematic chain implied by the active contacts.
//
//   cost = w · ‖J · z - r‖²
//   J    = [ (Gᵀ)⁺ · J_c_stack  |  0_{6 × n_λ} ]      ∈ R^{6 × n_vars}
//   r    = a_obj_des - (Gᵀ)⁺ · (J̇_c · v)_stack       ∈ R^6
//   z    = [a; λ]                                      (a in left nv slots)
//
// where a_obj_des = a_ff + Kp·e_pos + Kd·e_vel is the standard SE(3) PD
// reference (matching SE3Task's accumulator semantics).
//
// ⚠ v2 correction (guide §20): the *kinematic* Jacobian is (Gᵀ)⁺ · J_c, not
// G⁺ · J_c. G ∈ R^{6×n_λ} maps wrenches; (Gᵀ)⁺ ∈ R^{6×n_λ} maps stacked
// contact-frame *velocities* to object spatial velocity (the dual operator
// in twist space). Using G⁺ produces an n_λ-by-nv matrix — dimensional
// mismatch with the 6-row residual block.
//
// GraspCache caches both (Stage B-2): G_pinv() for force tasks,
// GTPinv() for this kinematic task.
//
// Activation gates (all must hold for ResidualDim() == 6):
//   1. ≥ 1 active contact (ActiveLambdaDim > 0) — without contacts the
//      object is not kinematically coupled to the robot.
//   2. ObjectStateProvider::IsValid() — watchdog gate (pose freshness).
//
// When either gate fails, ResidualDim() = 0 and ComputeResidual is a no-op.
//
// Mask: 6D axis selection [vx,vy,vz,wx,wy,wz] (mirrors SE3Task). Default =
// all six axes active. masked-out rows do not appear in the residual; the
// computed J / r are then 6×nv / 6 internally and rows are extracted into
// J_block / r_block by mask order (matches SE3Task::ComputeResidual).
//
// Caller wiring (DemoWbcController, full B-end wiring out of scope):
//   1. SetContactManager(ContactManagerConfig*)
//   2. SetContactManagerRuntime(ContactManager*)
//   3. SetGraspCache(GraspCache*)            — shared with Object/Internal force
//   4. SetObjectStateProvider(ObjectStateProvider*) — pose / twist / watchdog
//   5. SetSe3Reference(placement_des, v_des, a_ff)
//   6. SetGains(kp, kd)
//
// The task itself does *not* run GraspCache::Compute() or ContactManager::
// ComputeGraspMatrix() — those are the controller's once-per-tick
// responsibilities, shared with B-2/B-3 tasks.
// ────────────────────────────────────────────────
class ObjectSE3Task final : public TaskBase {
 public:
  [[nodiscard]] std::string_view Name() const noexcept override { return "object_se3"; }

  void Init(const pinocchio::Model& model, const RobotModelInfo& robot_info, PinocchioCache& cache,
            const YAML::Node& task_config) override;

  [[nodiscard]] int ResidualDim() const noexcept override { return current_residual_dim_; }

  void ComputeResidual(const PinocchioCache& cache, const ControlReference& ref,
                       const ContactState& contacts, int n_vars,
                       Eigen::Ref<Eigen::MatrixXd> J_block,
                       Eigen::Ref<Eigen::VectorXd> r_block) noexcept override;

  // Wire the manager config and size workspaces to (max_contact_vars, nv).
  // Init-time (RT-unsafe due to the resize).
  void SetContactManager(const ContactManagerConfig* manager) noexcept;

  void SetContactManagerRuntime(const ContactManager* mgr_rt) noexcept { mgr_rt_ = mgr_rt; }

  void SetGraspCache(const GraspCache* cache) noexcept { grasp_cache_ = cache; }

  void SetObjectStateProvider(const ObjectStateProvider* provider) noexcept {
    state_provider_ = provider;
  }

  // RT-safe references for the object PD.
  void SetSe3Reference(
      const pinocchio::SE3& placement_des,
      const Eigen::Matrix<double, 6, 1>& v_des = Eigen::Matrix<double, 6, 1>::Zero(),
      const Eigen::Matrix<double, 6, 1>& a_ff = Eigen::Matrix<double, 6, 1>::Zero()) noexcept;

  void SetGains(const Eigen::Matrix<double, 6, 1>& kp,
                const Eigen::Matrix<double, 6, 1>& kd) noexcept;

  // Update ResidualDim() based on contact activity AND provider validity.
  // Mirrors the one-tick-lag pattern of ForceTask / Object/Internal force
  // tasks: WQPFormulation polls ResidualDim() *before* ComputeResidual is
  // called, so the first tick after a contact toggle or validity flip
  // carries the prior dim. Caller may invoke this explicitly from the
  // controller's per-tick reset path; ComputeResidual also recomputes it
  // on entry.
  void UpdateResidualDim(const ContactState& contacts) noexcept;

 private:
  int nv_{0};
  int current_residual_dim_{0};
  int active_mask_count_{6};

  // 6D mask: true = axis controlled.
  std::array<bool, 6> mask_{};

  const ContactManagerConfig* manager_{nullptr};
  const ContactManager* mgr_rt_{nullptr};
  const GraspCache* grasp_cache_{nullptr};
  const ObjectStateProvider* state_provider_{nullptr};

  // PD gains (axis-wise; mirrors SE3Task convention).
  Eigen::Matrix<double, 6, 1> kp_{Eigen::Matrix<double, 6, 1>::Constant(100.0)};
  Eigen::Matrix<double, 6, 1> kd_{Eigen::Matrix<double, 6, 1>::Constant(20.0)};

  // Reference state (object-frame, world-aligned).
  pinocchio::SE3 placement_des_{pinocchio::SE3::Identity()};
  Eigen::Matrix<double, 6, 1> v_des_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> a_ff_{Eigen::Matrix<double, 6, 1>::Zero()};

  // Pre-allocated workspaces (sized in SetContactManager).
  //   jc_stack_   : [max_contact_vars × nv]   stacked contact Jacobians
  //   djv_stack_  : [max_contact_vars]        stacked J̇_c·v bias
  //   j_obj_full_ : [6 × nv]                  (Gᵀ)⁺ · J_c_stack
  //   r_full_     : [6]                       a_obj_des - (Gᵀ)⁺ · dJv_stack
  //   error_full_, v_error_full_, a_des_full_ : SE3 PD scratch
  Eigen::MatrixXd jc_stack_;
  Eigen::VectorXd djv_stack_;
  Eigen::MatrixXd j_obj_full_;
  Eigen::Matrix<double, 6, 1> r_full_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> error_full_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> v_error_full_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> a_des_full_{Eigen::Matrix<double, 6, 1>::Zero()};
};

}  // namespace rtc::tsid
