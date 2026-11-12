#pragma once

#include "rtc_tsid/constraints/contact_constraint.hpp"
#include "rtc_tsid/controller/tsid_controller.hpp"
#include "rtc_tsid/locomotion/reference_kinematics.hpp"
#include "rtc_tsid/tasks/com_task.hpp"
#include "rtc_tsid/tasks/force_task.hpp"
#include "rtc_tsid/tasks/posture_task.hpp"
#include "rtc_tsid/tasks/se3_task.hpp"
#include "rtc_tsid/types/wbc_types.hpp"

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

namespace rtc::tsid {

// ────────────────────────────────────────────────
// The whole-body QP of a legged robot: one TSIDController whose tasks and
// constraints come from a `whole_body_qp:` map, assembled through the YAML
// dispatch (formulation/task_factory.hpp). Robot-agnostic: the contacts (how
// many, which frames, point or surface) and the torso frame come from a
// LocomotionModelSpec the owner builds from its model configuration; the
// task roles ("feet", "torso") are names, not sides.
//
// Built:
//   posture       PostureTask on the full tangent (base: velocity damping only)
//   feet          one SE3Task per contact (world axes), ACTIVE ONLY WHILE ITS
//                 CONTACT IS NOT — a stance foot is held by the contact rows
//                 (WD-59)
//   torso         SE3Task on spec.torso_frame (world axes, `mask` selects axes)
//   force         ForceTask toward ControlReference::lambda_des
//   com           CoMTask (optional key)
//   eom · contact (ContactConstraint, Baumgarte keys) · friction_cone ·
//   joint_limit · torque_limit — the rtc_tsid constraints with their keys
//
// Per tick (RT, allocation-free): UpdateReferences — the reference's forward
// kinematics (ReferenceKinematics) to the foot / torso / CoM tasks, each foot
// task's activity from its contact, the force task's row count from the
// contacts (before the solve, so a touch-down tick has its rows), then the
// controller's Compute. Posture and force read the ControlReference directly.
//
// YAML (`whole_body_qp:`; an unknown key here or in contact / overrides is
// refused, a task / constraint map takes its own task's keys):
//   formulation_type: wqp | hqp
//   solver: {…}                       # ParseQPSolverConfig keys
//   contact: {friction_coeff, friction_faces, patch_half_length_x,
//             patch_half_length_y, torsional_friction_coeff,
//             normal_force_min, normal_force_max, t_ramp}   # every contact
//   contact_overrides: {<contact name>: {…same keys but t_ramp}}  # optional
//   tasks:       {posture: {…}, feet: {…}, torso: {…}, force: {…}, com: {…}?}
//   constraints: {eom: {}, contact: {…}, friction_cone: {}, joint_limit: {…},
//                 torque_limit: {…}}
// Each task / constraint map takes its rtc_tsid keys (kp, kd, weight,
// priority, mask; baumgarte, terrain_height; position_margin, …), never type /
// name / frame / base_frame (the builder sets those). eom and contact are
// required; the other constraints are optional.
// ────────────────────────────────────────────────

struct LocomotionContactSpec {
  std::string name;   // the contact's name (contact_overrides key)
  std::string frame;  // its model frame
  int dim{6};         // 6 = surface (wrench), 3 = point (force)
};

struct LocomotionModelSpec {
  std::vector<LocomotionContactSpec> contacts;  // order = contact index everywhere
  std::string torso_frame;
};

class LocomotionFormulationBuilder {
 public:
  LocomotionFormulationBuilder() = default;
  // The built terms point at contacts_: the builder stays where it was built.
  LocomotionFormulationBuilder(const LocomotionFormulationBuilder&) = delete;
  LocomotionFormulationBuilder& operator=(const LocomotionFormulationBuilder&) = delete;
  LocomotionFormulationBuilder(LocomotionFormulationBuilder&&) = delete;
  LocomotionFormulationBuilder& operator=(LocomotionFormulationBuilder&&) = delete;

  // The contact frame ids in spec order — what the owner's PinocchioCache::Init
  // takes (cache.contact_frames[i] ↔ contact i). Throws for an unknown frame.
  [[nodiscard]] static std::vector<pinocchio::FrameIndex> ContactFrameIds(
      const pinocchio::Model& model, const LocomotionModelSpec& spec);

  // Init time (allocates, throws std::invalid_argument). `cache` must be Init'd
  // with ContactFrameIds(model, spec) and not yet locked (the tasks register
  // their frames); `control_period` > 0 is the owner's 1/control_rate.
  void Init(const pinocchio::Model& model, const RobotModelInfo& robot_info, PinocchioCache& cache,
            const LocomotionModelSpec& spec, const YAML::Node& whole_body_qp,
            double control_period);

  // A ContactState for this contact set: sized, normals seeded, every contact
  // active (fully), t_ramp from the YAML.
  [[nodiscard]] ContactState MakeContactState() const;

  // Push the reference to the frame tasks and the contact-dependent state
  // (header). false when the reference's kinematics could not be evaluated
  // (sizes, non-finite, a zero quaternion) — the frame tasks keep the last
  // reference. RT-safe.
  [[nodiscard]] bool UpdateReferences(const ControlReference& ref,
                                      const ContactState& contacts) noexcept;

  // UpdateReferences, then the controller's Compute — or, when the reference
  // was refused, TSIDController::SkipTick (no solve, tau_valid false). RT-safe.
  [[nodiscard]] const CommandOutput& Compute(const ControlState& state, const ControlReference& ref,
                                             const PinocchioCache& cache,
                                             const ContactState& contacts) noexcept;

  [[nodiscard]] TSIDController& Controller() noexcept { return controller_; }

  [[nodiscard]] const ContactManagerConfig& Contacts() const noexcept { return contacts_; }

  [[nodiscard]] std::size_t NumFeet() const noexcept { return feet_.size(); }

  [[nodiscard]] SE3Task* FootTask(std::size_t i) const noexcept { return feet_[i]; }

  [[nodiscard]] SE3Task* TorsoTask() const noexcept { return torso_; }

  [[nodiscard]] PostureTask* Posture() const noexcept { return posture_; }

  [[nodiscard]] ForceTask* Force() const noexcept { return force_; }

  [[nodiscard]] CoMTask* Com() const noexcept { return com_; }

  [[nodiscard]] ContactConstraint* ContactRows() const noexcept { return contact_rows_; }

  [[nodiscard]] const ReferenceKinematics& Kinematics() const noexcept { return kinematics_; }

  [[nodiscard]] double RampTime() const noexcept { return t_ramp_; }

 private:
  TSIDController controller_;
  ContactManagerConfig contacts_;
  ReferenceKinematics kinematics_;  // frames: feet (contact order), then the torso
  std::vector<SE3Task*> feet_;      // owned by the formulation
  SE3Task* torso_{nullptr};
  PostureTask* posture_{nullptr};
  ForceTask* force_{nullptr};
  CoMTask* com_{nullptr};
  ContactConstraint* contact_rows_{nullptr};
  double t_ramp_{0.0};
};

}  // namespace rtc::tsid
