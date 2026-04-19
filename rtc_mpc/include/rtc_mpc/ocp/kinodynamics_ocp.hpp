#ifndef RTC_MPC_OCP_KINODYNAMICS_OCP_HPP_
#define RTC_MPC_OCP_KINODYNAMICS_OCP_HPP_

/// @file kinodynamics_ocp.hpp
/// @brief Concrete `OCPHandlerBase` for v2.2-sense "KinoDynamics MPC" on a
///        fixed-base manipulator.
///
/// Class naming reflects our MPC mode (kinematics-level control) per the
/// v2.2 architecture; the backing Aligator class is
/// `MultibodyConstraintFwdDynamicsTpl` (see
/// `docs/mpc_implementation_progress.md` §"Phase 3 Spike Notes" — the
/// Aligator-named `KinodynamicsFwdDynamicsTpl` is a floating-base
/// centroidal class unsuitable for fixed-base manipulators).
///
/// Layout:
/// - state  `x = [q; v]` ∈ R^{nq+nv}
/// - control `u = τ` ∈ R^{nv} (identity actuation; contact forces emerge
///   from the rigid-contact proximal solve as Lagrange multipliers, NOT
///   part of `u`)
/// - Riccati gain from Aligator: `K ∈ R^{nu × ndx}` with `ndx = 2·nv`;
///   `rtc::mpc::RiccatiFeedback::SetGain(K, nu, nx)` with `accel_only`
///   reads all of `K` since `nu = nv` (fixed-base).
///
/// Phase-3 scope intentionally omits joint-box / torque-box / friction-cone
/// constraints — rigid-contact dynamics alone enforces non-negative normal
/// force via the proximal solve, which is sufficient for the initial
/// end-to-end proof. Additive constraints land in a follow-up iteration
/// (tracked in the progress doc's "Risks" section).

#include "rtc_mpc/ocp/cost_factory.hpp"
#include "rtc_mpc/ocp/ocp_handler_base.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/modelling/multibody/frame-placement.hpp>
#include <aligator/modelling/state-error.hpp>
#include <pinocchio/algorithm/contact-info.hpp>
#include <pinocchio/algorithm/proximal.hpp>
#pragma GCC diagnostic pop

#include <memory>
#include <string_view>
#include <vector>

namespace rtc::mpc {

/// @brief Non-owning raw-pointer handles to residuals stored inside a
///        StageModel's polymorphic cost tree. Populated **after** problem
///        assembly; used by `UpdateReferences` for alloc-free mutation.
///
/// Null entries indicate the corresponding weight was <= 0 at Build and
/// the residual is absent from the stage (no lookup possible).
struct KinoStageHandles {
  aligator::FramePlacementResidualTpl<double> *frame_placement{nullptr};
  aligator::StateErrorResidualTpl<double> *state_reg{nullptr};
  aligator::ControlErrorResidualTpl<double> *control_reg{nullptr};
};

class KinoDynamicsOCP : public OCPHandlerBase {
public:
  KinoDynamicsOCP() = default;
  ~KinoDynamicsOCP() override = default;

  KinoDynamicsOCP(const KinoDynamicsOCP &) = delete;
  KinoDynamicsOCP &operator=(const KinoDynamicsOCP &) = delete;
  KinoDynamicsOCP(KinoDynamicsOCP &&) = delete;
  KinoDynamicsOCP &operator=(KinoDynamicsOCP &&) = delete;

  [[nodiscard]] OCPBuildError Build(const PhaseContext &ctx,
                                    const RobotModelHandler &model,
                                    const OCPLimits &limits) noexcept override;

  [[nodiscard]] OCPBuildError
  UpdateReferences(const PhaseContext &ctx) noexcept override;

  [[nodiscard]] bool Built() const noexcept override {
    return problem_ != nullptr;
  }

  [[nodiscard]] aligator::TrajOptProblemTpl<double> &problem() override {
    return *problem_;
  }

  [[nodiscard]] int horizon_length() const noexcept override {
    return horizon_length_;
  }

  [[nodiscard]] std::string_view ocp_type() const noexcept override {
    return std::string_view{"kinodynamics"};
  }

private:
  // Problem tree (ownership)
  std::unique_ptr<aligator::TrajOptProblemTpl<double>> problem_{};

  // Per-stage cached handles; size == horizon_length_ after Build.
  std::vector<KinoStageHandles> stage_handles_{};
  KinoStageHandles terminal_handles_{};

  // Cached topology + limits — used by UpdateReferences to detect changes
  // that would require a full Build.
  int horizon_length_{0};
  double dt_{0.0};
  int nq_{0};
  int nv_{0};
  int nu_{0};
  std::vector<std::vector<int>> stage_active_contacts_{};

  // Dynamics bookkeeping (kept so Build state can be inspected; not reused
  // across Build calls — Build always rebuilds from scratch).
  Eigen::MatrixXd actuation_matrix_{};
};

} // namespace rtc::mpc

#endif // RTC_MPC_OCP_KINODYNAMICS_OCP_HPP_
