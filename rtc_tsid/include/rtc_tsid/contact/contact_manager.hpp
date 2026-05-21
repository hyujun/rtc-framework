#pragma once

#include "rtc_tsid/types/object_frame.hpp"
#include "rtc_tsid/types/wbc_types.hpp"

#include <Eigen/Core>

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Stage B-1: ContactManager — assembles object-level structures from the
// per-contact pieces produced by PinocchioCache.
//
// Concretely it provides three pure-function-style operations used by
// Stage B-2+ tasks (ObjectWrenchTask, InternalForceTask, ObjectSE3Task):
//
//   ┌────────────────────────────┬─────────────────────────────────────┐
//   │ ActiveLambdaDim()          │ Σ_{i active} contact_dim_i          │
//   │                            │ (current λ vector size; differs     │
//   │                            │  from manager.max_contact_vars)     │
//   ├────────────────────────────┼─────────────────────────────────────┤
//   │ ComputeGraspMatrix(...)    │ G ∈ R^{6 × n_λ_active}              │
//   │                            │ Maps stacked contact wrenches λ to  │
//   │                            │ the net wrench at the object origin │
//   │                            │ p_o. Point/Surface contacts both    │
//   │                            │ supported (3-col / 6-col blocks).   │
//   ├────────────────────────────┼─────────────────────────────────────┤
//   │ StackContactJacobians(...) │ J_c_stack ∈ R^{n_λ_active × nv}     │
//   │                            │ Vertical stack of per-contact       │
//   │                            │ contact Jacobians (top-3 rows for   │
//   │                            │ point, top-6 for surface).          │
//   └────────────────────────────┴─────────────────────────────────────┘
//
// Init() pre-allocates two workspace buffers sized to the worst case
// (max_contact_vars × 6 and max_contact_vars × nv). The Compute*
// methods write into Eigen::Ref views over caller-owned matrices (so the
// caller can resize the output up-front and reuse it across ticks).
//
// RT-safe usage pattern:
//   - Call Init() once at lifecycle configure.
//   - At each tick, pre-resize G_out / J_out to active dim (no alloc if
//     the workspace inside the caller is already a fixed-shape buffer),
//     then call Compute*().
//
// Inactive contacts (ContactState::Entry::active == false) are skipped:
// they do not contribute rows/cols. Active contacts' rows/cols appear in
// the output in their config index order (matches the convention used by
// ForceTask/ContactConstraint).
//
// NOTE: G is built in a world-aligned basis (LOCAL_WORLD_ALIGNED contact
// Jacobians), matching rtc_tsid's convention. The ObjectFrame's rotation
// R_w is not consumed here — reserved for Stage B-2+ wrench variants.
// ────────────────────────────────────────────────
class ContactManager {
 public:
  // Pre-allocate workspaces for the worst case implied by `manager_cfg`.
  // Safe to call once at lifecycle configure; RT-safe Compute* afterwards.
  void Init(const ContactManagerConfig& manager_cfg, int nv) noexcept;

  // Σ of contact_dim_i over active contacts. RT-safe.
  [[nodiscard]] int ActiveLambdaDim(const ContactState& contacts) const noexcept;

  // Write G[6 × ActiveLambdaDim] into G_out. Caller must size G_out to
  // (6, ActiveLambdaDim(contacts)) before calling. Inactive contacts are
  // skipped; active contacts contribute a 6×cdim column block at their
  // active offset (cumulative over preceding active contacts).
  //
  // Convention: world-aligned wrenches expressed at p_o (object_frame.p_w).
  // Point contact (cdim=3): force-only → 6×3 block via PointGraspBlock.
  // Surface contact (cdim=6): full wrench → 6×6 block via
  // AdjointDualWorldAligned.
  //
  // p_ci is taken from cache.contact_frames[i].oMf.translation() (the
  // Pinocchio-computed world placement of contact frame i).
  //
  // RT-safe: stack-only arithmetic on fixed 6×6 / 6×3 blocks.
  void ComputeGraspMatrix(const PinocchioCache& cache, const ContactState& contacts,
                          const ObjectFrame& object_frame,
                          Eigen::Ref<Eigen::MatrixXd> G_out) const noexcept;

  // Write J_c_stack[ActiveLambdaDim × nv] into J_out. Caller must size
  // J_out to (ActiveLambdaDim(contacts), nv) before calling. Inactive
  // contacts are skipped; active contacts contribute cdim rows from the
  // top of their contact-frame Jacobian (point: 3, surface: 6).
  //
  // RT-safe: block copy from cache.contact_frames[i].J.
  void StackContactJacobians(const PinocchioCache& cache, const ContactState& contacts,
                             Eigen::Ref<Eigen::MatrixXd> J_out) const noexcept;

  // Write the stacked (J̇_c · v) bias for active contacts into v_out.
  // Caller must size v_out to (ActiveLambdaDim(contacts)) before calling.
  // Inactive contacts skip cdim entries; active contacts contribute the
  // top-cdim rows of cache.contact_frames[i].dJv (the classical-acceleration
  // bias of the contact frame, LOCAL_WORLD_ALIGNED).
  //
  // Used by Stage B-4 ObjectSE3Task to assemble the reference
  //   r = a_obj_des - (Gᵀ)⁺ · (J̇_c·v)_stack.
  //
  // RT-safe: segment copy from cache.contact_frames[i].dJv.
  void StackContactJDotV(const PinocchioCache& cache, const ContactState& contacts,
                         Eigen::Ref<Eigen::VectorXd> v_out) const noexcept;

  [[nodiscard]] const ContactManagerConfig* config() const noexcept { return manager_cfg_; }

 private:
  const ContactManagerConfig* manager_cfg_{nullptr};
  int nv_{0};
};

}  // namespace rtc::tsid
