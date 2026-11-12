#pragma once

#include "rtc_tsid/core/constraint_base.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/spatial/se3.hpp>
#pragma GCC diagnostic pop

#include <vector>

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Contact acceleration constraint
//
// 각 active contact i: Jc_i · a + dJc_i · v = a_c*_i
//   point: Jc_i = J[:3, :] → 3행
//   surface: Jc_i = J[:6, :] → 6행
//
// eq_dim = Σ active_contact_dim_i
// ineq_dim = 0
//
// a_c* is zero unless the Baumgarte stabilisation is configured (YAML below):
// then a_c* = −Kp ⊙ e − Kd ⊙ v_c with e = [p − p*, log3(R R*ᵀ)] (world axes,
// the separated SplitWorld error), v_c the contact frame's LOCAL_WORLD_ALIGNED
// twist (J·v), and the anchor (p*, R*) the frame's pose latched on the tick the
// contact became active (`active` false → true; a contact active from Init
// latches on the first Compute). The anchor stays until the contact goes
// inactive and comes back, Reset(), or the constraint itself is turned back
// on by a phase preset (while it was off nothing watched the contacts, so the
// feet may have moved: every anchor is dropped and latched anew). A pose that
// is not finite on the edge tick is not latched (the term is skipped that
// tick; the latch waits for a finite pose). `terrain_height` (optional)
// replaces the latched z so the foot is held at the ground, not where it
// touched. A point contact uses the first three rows (and gains). With every
// gain zero (or no `baumgarte:` key) the term is skipped entirely — same
// bytes as before, no latch. Near a rotation error of π log3 flips its axis
// (the SplitWorld cut): a stance foot does not get there.
//
// YAML (`contact_accel`, every key optional):
//   baumgarte: {kp: <scalar | 6>, kd: <scalar | 6>}   # ≥ 0, finite
//   terrain_height: 0.0                                # [m]
// ────────────────────────────────────────────────
class ContactConstraint final : public ConstraintBase {
 public:
  [[nodiscard]] std::string_view Name() const noexcept override { return "contact_accel"; }

  void Init(const pinocchio::Model& model, const RobotModelInfo& robot_info, PinocchioCache& cache,
            const YAML::Node& constraint_config) override;

  [[nodiscard]] int EqDim(const ContactState& contacts) const noexcept override;
  [[nodiscard]] int IneqDim(const ContactState& contacts) const noexcept override;
  [[nodiscard]] int MaxEqDim(const ContactManagerConfig& contacts) const noexcept override;
  [[nodiscard]] int MaxIneqDim(const ContactManagerConfig& contacts) const noexcept override;

  void ComputeEquality(const PinocchioCache& cache, const ContactState& contacts,
                       const RobotModelInfo& robot_info, int n_vars,
                       Eigen::Ref<Eigen::MatrixXd> A_block,
                       Eigen::Ref<Eigen::VectorXd> b_block) noexcept override;

  void ComputeInequality(const PinocchioCache& cache, const ContactState& contacts,
                         const RobotModelInfo& robot_info, int n_vars,
                         Eigen::Ref<Eigen::MatrixXd> C_block, Eigen::Ref<Eigen::VectorXd> l_block,
                         Eigen::Ref<Eigen::VectorXd> u_block) noexcept override;

  // Drop every latched anchor: the next Compute latches each active contact anew.
  void Reset() noexcept override;

  void OnEnabled() noexcept override { Reset(); }

  // ContactManagerConfig 참조 설정 (init 후 호출) — allocates the per-contact
  // anchor slots (not RT).
  void SetContactManager(const ContactManagerConfig* manager);

  [[nodiscard]] bool BaumgarteEnabled() const noexcept { return baumgarte_; }

  // The anchor of contact `i` and whether one is latched (diagnostics / tests).
  [[nodiscard]] bool AnchorLatched(std::size_t i) const noexcept {
    return i < slots_.size() && slots_[i].latched;
  }

  [[nodiscard]] const pinocchio::SE3& Anchor(std::size_t i) const noexcept {
    return slots_[i].anchor;
  }

  // a_c* of contact `i` on the last ComputeEquality (zero when inactive or off).
  [[nodiscard]] const Eigen::Matrix<double, 6, 1>& TargetAcceleration(
      std::size_t i) const noexcept {
    return slots_[i].a_star;
  }

 private:
  int nv_{0};
  const ContactManagerConfig* manager_{nullptr};
  bool baumgarte_{false};
  bool use_terrain_{false};
  double terrain_height_{0.0};
  Eigen::Matrix<double, 6, 1> kp_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> kd_{Eigen::Matrix<double, 6, 1>::Zero()};

  // Per contact slot [max_contacts]: the anchor latched while active and the
  // last a_c* (zero when inactive, off, or waiting for a finite pose).
  struct Slot {
    pinocchio::SE3 anchor{pinocchio::SE3::Identity()};
    Eigen::Matrix<double, 6, 1> a_star{Eigen::Matrix<double, 6, 1>::Zero()};
    bool latched{false};
  };

  std::vector<Slot> slots_;
};

}  // namespace rtc::tsid
