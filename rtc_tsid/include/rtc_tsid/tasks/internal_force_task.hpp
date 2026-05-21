#pragma once

#include "rtc_tsid/contact/contact_manager.hpp"
#include "rtc_tsid/contact/grasp_cache.hpp"
#include "rtc_tsid/core/task_base.hpp"

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Stage B-3: InternalForceTask — drives internal (squeeze) forces toward a
// projected reference that *only lives inside Null(G)*.
//
//   cost  = w · ‖J·z - r‖²
//   J     = [0_{n_λ × nv} | I_{n_λ}]   (one row per active λ slot)
//   r     = P_N · λ_squeeze_des         (active-packed)
//
// The reference projection is the v2 fix vs. the original guide: passing
// the raw λ_squeeze_des would partially fight ObjectWrenchTask (because
// any Range(Gᵀ) component contributes to the net object wrench). P_N
// projects away that conflict so the two tasks operate on orthogonal
// subspaces.
//
// Activation: rank-based nullity. The task runs only when the grasp has a
// non-trivial null space:
//   nullity = n_λ_active - rank_G
//   if nullity ≤ 0  →  ResidualDim() = 0 (skip)
//
// The naive "n_λ ≤ 6 → skip" check from v1 of the guide is *wrong* for
// degenerate configurations (e.g. collinear fingers): n_λ might exceed 6
// while rank_G drops below 6, leaving a valid internal-force subspace
// that the naive check would miss. Always consult rank_G from GraspCache.
//
// Caller wiring (DemoWbcController, B-end aggregate wiring out of scope):
//   1. SetContactManager(manager_cfg)
//   2. SetContactManagerRuntime(ContactManager*)
//   3. SetGraspCache(GraspCache*)           — shared with ObjectWrenchTask
//   4. SetSqueezeReference(λ_squeeze_des)   — max_contact_vars sized
//
// The task does *not* run GraspCache::Compute() — that's the controller's
// once-per-tick responsibility. It only reads GraspCache::ProjN(),
// CurrentLambdaDim(), Rank().
// ────────────────────────────────────────────────
class InternalForceTask final : public TaskBase {
 public:
  [[nodiscard]] std::string_view Name() const noexcept override { return "internal_force"; }

  void Init(const pinocchio::Model& model, const RobotModelInfo& robot_info, PinocchioCache& cache,
            const YAML::Node& task_config) override;

  [[nodiscard]] int ResidualDim() const noexcept override { return current_residual_dim_; }

  void ComputeResidual(const PinocchioCache& cache, const ControlReference& ref,
                       const ContactState& contacts, int n_vars,
                       Eigen::Ref<Eigen::MatrixXd> J_block,
                       Eigen::Ref<Eigen::VectorXd> r_block) noexcept override;

  // Wire the manager config and size the workspace to max_contact_vars.
  // Init-time (RT-unsafe).
  void SetContactManager(const ContactManagerConfig* manager) noexcept;

  void SetContactManagerRuntime(const ContactManager* mgr_rt) noexcept { mgr_rt_ = mgr_rt; }

  void SetGraspCache(const GraspCache* cache) noexcept { grasp_cache_ = cache; }

  // RT-safe: copies into local max_contact_vars-sized buffer. Caller
  // passes the *full* reference in slot-offset layout (matches
  // ControlReference::lambda_des). Inactive slot entries are ignored.
  void SetSqueezeReference(const Eigen::VectorXd& lambda_squeeze_des) noexcept;

  void UpdateResidualDim(const ContactState& contacts) noexcept;

 private:
  int nv_{0};
  int current_residual_dim_{0};

  const ContactManagerConfig* manager_{nullptr};
  const ContactManager* mgr_rt_{nullptr};
  const GraspCache* grasp_cache_{nullptr};

  // Caller-provided desired squeeze (slot-offset layout, max_contact_vars).
  Eigen::VectorXd lambda_squeeze_des_;
  bool has_reference_{false};

  // Active-packed scratch (P_N · λ_des_active). Sized to max_contact_vars
  // at Init; only the top n_lambda_active entries are used per tick.
  Eigen::VectorXd lambda_des_active_;
  Eigen::VectorXd r_projected_;
};

}  // namespace rtc::tsid
