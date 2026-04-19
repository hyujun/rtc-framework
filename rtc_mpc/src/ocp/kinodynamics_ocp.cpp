#include "rtc_mpc/ocp/kinodynamics_ocp.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <aligator/core/stage-model.hpp>
#include <aligator/modelling/costs/quad-residual-cost.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <aligator/modelling/dynamics/integrator-semi-euler.hpp>
#include <aligator/modelling/dynamics/multibody-constraint-fwd.hpp>
#include <aligator/modelling/spaces/multibody.hpp>
#pragma GCC diagnostic pop

#include <algorithm>
#include <exception>
#include <string>
#include <utility>

namespace rtc::mpc {

namespace {

using CostStack = aligator::CostStackTpl<double>;
using QuadCost = aligator::QuadraticResidualCostTpl<double>;
using FrameRes = aligator::FramePlacementResidualTpl<double>;
using StateRes = aligator::StateErrorResidualTpl<double>;
using ControlRes = aligator::ControlErrorResidualTpl<double>;
using PhaseSpace = aligator::MultibodyPhaseSpace<double>;
using MultibodyConstraintFwd =
    aligator::dynamics::MultibodyConstraintFwdDynamicsTpl<double>;
using IntegratorSE = aligator::dynamics::IntegratorSemiImplEulerTpl<double>;
using StageModel = aligator::StageModelTpl<double>;
using TrajOptProblem = aligator::TrajOptProblemTpl<double>;

using RigidConstraintModel = pinocchio::RigidConstraintModel;
using RigidConstraintVec = PINOCCHIO_ALIGNED_STD_VECTOR(RigidConstraintModel);

// Stage k has time t_k = k*dt. Returns the active contact frame ids at that
// time per `ContactPlan::phases`; empty if no phase covers t_k (free-flight)
// or if `phases` is empty. Sets `*overlap_found = true` if more than one
// phase's [t_start, t_end) contains t_k.
std::vector<int> ActiveContactsAtTime(const ContactPlan &plan, double t,
                                      bool *overlap_found) noexcept {
  std::vector<int> active{};
  int cover_count = 0;
  for (const auto &phase : plan.phases) {
    if (t >= phase.t_start && t < phase.t_end) {
      ++cover_count;
      if (cover_count == 1) {
        active = phase.active_frame_ids; // copy
      } else {
        if (overlap_found != nullptr) {
          *overlap_found = true;
        }
      }
    }
  }
  return active;
}

// Build the rigid-contact models vector for a given active-contact set.
// Each entry references a frame from the Pinocchio model; placement is
// taken from the frame itself. Contact type is CONTACT_3D (point) or
// CONTACT_6D (wrench) per `RobotModelHandler::contact_frames()[i].dim`.
RigidConstraintVec
BuildConstraintModels(const RobotModelHandler &model,
                      const std::vector<int> &active_frame_ids) noexcept {
  RigidConstraintVec out{};
  const auto &pin_model = model.model();
  const auto &frames = model.contact_frames();

  for (int fid : active_frame_ids) {
    // Locate the matching ContactFrameInfo to read its dim.
    int dim = 3;
    std::string name = "contact_" + std::to_string(fid);
    for (const auto &info : frames) {
      if (info.frame_id == fid) {
        dim = info.dim;
        name = info.name;
        break;
      }
    }
    const auto joint_id = static_cast<pinocchio::JointIndex>(
        pin_model.frames[static_cast<std::size_t>(fid)].parentJoint);
    const auto &placement =
        pin_model.frames[static_cast<std::size_t>(fid)].placement;
    const auto contact_type =
        (dim == 6) ? pinocchio::CONTACT_6D : pinocchio::CONTACT_3D;
    out.emplace_back(contact_type, pin_model, joint_id, placement,
                     pinocchio::LOCAL);
    out.back().name = name;
  }
  return out;
}

// Walk a freshly-built StageModel's polymorphic cost tree and populate
// raw-pointer handles for alloc-free mutation in UpdateReferences.
KinoStageHandles CacheStageHandles(StageModel &stage,
                                   const StageComponentKeys &keys) noexcept {
  KinoStageHandles h{};
  auto *stack = stage.getCost<CostStack>();
  if (stack == nullptr) {
    return h;
  }

  if (keys.has_frame_placement) {
    auto *quad =
        stack->getComponent<QuadCost>(std::string(kCostKeyFramePlacement));
    if (quad != nullptr) {
      h.frame_placement = quad->getResidual<FrameRes>();
    }
  }
  if (keys.has_state_reg) {
    auto *quad = stack->getComponent<QuadCost>(std::string(kCostKeyStateReg));
    if (quad != nullptr) {
      h.state_reg = quad->getResidual<StateRes>();
    }
  }
  if (keys.has_control_reg) {
    auto *quad = stack->getComponent<QuadCost>(std::string(kCostKeyControlReg));
    if (quad != nullptr) {
      h.control_reg = quad->getResidual<ControlRes>();
    }
  }
  return h;
}

// Mutate cached handles to reflect new references. Pre-condition: topology
// unchanged (caller verified); handles non-null only where corresponding
// components were built.
void ApplyReferences(const KinoStageHandles &h, const PhaseCostConfig &cfg,
                     const pinocchio::SE3 &ee_target, int nq, int nv) noexcept {
  if (h.frame_placement != nullptr) {
    h.frame_placement->setReference(ee_target);
  }
  if (h.state_reg != nullptr) {
    // Aligator `StateErrorResidual::target_` is a public VectorXs field.
    Eigen::VectorXd x_target(nq + nv);
    x_target.head(nq) = cfg.q_posture_ref;
    x_target.tail(nv).setZero();
    h.state_reg->target_ = x_target;
  }
  if (h.control_reg != nullptr) {
    // Target is zero control; preserved across phase changes.
    // (Left alone; no mutation needed unless u_ref becomes non-zero.)
  }
}

} // namespace

OCPBuildError KinoDynamicsOCP::Build(const PhaseContext &ctx,
                                     const RobotModelHandler &model,
                                     const OCPLimits &limits) noexcept {
  // ── Input validation ────────────────────────────────────────────────
  if (!model.Initialised()) {
    return OCPBuildError::kModelNotInitialised;
  }
  if (ctx.ocp_type != "kinodynamics") {
    return OCPBuildError::kInvalidPhaseContext;
  }
  const auto &cfg = ctx.cost_config;
  if (cfg.horizon_length <= 0 || cfg.dt <= 0.0) {
    return OCPBuildError::kInvalidCostConfig;
  }

  const int nq = model.nq();
  const int nv = model.nv();
  const int nu = nv; // fixed-base, identity actuation

  if (cfg.q_posture_ref.size() != nq) {
    return OCPBuildError::kInvalidCostConfig;
  }
  if (limits.u_min.size() != 0 && limits.u_min.size() != nu) {
    return OCPBuildError::kLimitsDimMismatch;
  }
  if (limits.u_max.size() != 0 && limits.u_max.size() != nu) {
    return OCPBuildError::kLimitsDimMismatch;
  }

  // Cross-check contact plan against model frames.
  const auto &model_frames = model.contact_frames();
  auto frame_is_known = [&](int fid) {
    return std::any_of(
        model_frames.begin(), model_frames.end(),
        [fid](const ContactFrameInfo &f) { return f.frame_id == fid; });
  };
  for (const auto &phase : ctx.contact_plan.phases) {
    for (int fid : phase.active_frame_ids) {
      if (!frame_is_known(fid)) {
        return OCPBuildError::kContactPlanModelMismatch;
      }
    }
  }

  // Resolve per-stage active contacts; detect overlap.
  const int H = cfg.horizon_length;
  std::vector<std::vector<int>> stage_active(static_cast<std::size_t>(H));
  bool overlap = false;
  for (int k = 0; k < H; ++k) {
    const double t_k = static_cast<double>(k) * cfg.dt;
    stage_active[static_cast<std::size_t>(k)] =
        ActiveContactsAtTime(ctx.contact_plan, t_k, &overlap);
  }
  if (overlap) {
    return OCPBuildError::kOverlappingContactPhases;
  }

  // ── Problem assembly (Aligator ctors may throw → wrap) ─────────────
  try {
    PhaseSpace space(model.model());
    Eigen::MatrixXd actuation = Eigen::MatrixXd::Identity(nv, nv);
    pinocchio::ProximalSettings prox_settings(1e-9, 1e-10, 10);

    std::vector<xyz::polymorphic<StageModel>> stages{};
    stages.reserve(static_cast<std::size_t>(H));

    // Keys per stage — cached for later handle lookup AFTER problem
    // construction. Caching raw pointers on this local `stages` vector
    // would dangle because TrajOptProblem copies the vector into its own
    // `stages_` member.
    std::vector<StageComponentKeys> stage_keys{};
    stage_keys.reserve(static_cast<std::size_t>(H));

    for (int k = 0; k < H; ++k) {
      const auto &active_fids = stage_active[static_cast<std::size_t>(k)];

      // Build running cost via CostFactory.
      CostFactoryError cost_err = CostFactoryError::kNoError;
      auto stage_cost =
          cost_factory::BuildRunningCost(cfg, model, ctx.ee_target, &cost_err);
      if (cost_err != CostFactoryError::kNoError) {
        return OCPBuildError::kInvalidCostConfig;
      }

      // Build rigid-contact dynamics for this stage.
      auto constraint_models = BuildConstraintModels(model, active_fids);
      MultibodyConstraintFwd cont_dyn(space, actuation, constraint_models,
                                      prox_settings);
      IntegratorSE integrator(
          xyz::polymorphic<aligator::dynamics::ODEAbstractTpl<double>>(
              cont_dyn),
          cfg.dt);

      // Compose stage (cost + dynamics are value-copied into polymorphic).
      StageModel stage(stage_cost.stack, integrator);
      stages.emplace_back(stage);
      stage_keys.push_back(stage_cost.keys);
    }

    // Terminal cost (no control reg).
    CostFactoryError term_err = CostFactoryError::kNoError;
    auto term_cost =
        cost_factory::BuildTerminalCost(cfg, model, ctx.ee_target, &term_err);
    if (term_err != CostFactoryError::kNoError) {
      return OCPBuildError::kInvalidCostConfig;
    }

    // Build problem with initial state at model neutral (caller will
    // override via `problem().setInitState(x0)` from sensor data).
    Eigen::VectorXd x0 = space.neutral();

    auto problem_new =
        std::make_unique<TrajOptProblem>(x0, stages, term_cost.stack);

    // Walk the STORED stages inside the freshly built problem to cache
    // raw handles. Pointers into `stages` (local) would dangle — the
    // TrajOptProblem ctor copied that vector into `problem_new->stages_`.
    std::vector<KinoStageHandles> fresh_handles{};
    fresh_handles.reserve(static_cast<std::size_t>(H));
    for (int k = 0; k < H; ++k) {
      auto &stored_stage = *problem_new->stages_[static_cast<std::size_t>(k)];
      fresh_handles.push_back(CacheStageHandles(
          stored_stage, stage_keys[static_cast<std::size_t>(k)]));
    }

    // Walk terminal cost for handles (term_cost_ is polymorphic-stored
    // inside TrajOptProblem).
    KinoStageHandles term_handles{};
    auto *term_stack = dynamic_cast<CostStack *>(&*problem_new->term_cost_);
    if (term_stack != nullptr) {
      if (term_cost.keys.has_frame_placement) {
        auto *quad = term_stack->getComponent<QuadCost>(
            std::string(kCostKeyFramePlacement));
        if (quad != nullptr) {
          term_handles.frame_placement = quad->getResidual<FrameRes>();
        }
      }
      if (term_cost.keys.has_state_reg) {
        auto *quad =
            term_stack->getComponent<QuadCost>(std::string(kCostKeyStateReg));
        if (quad != nullptr) {
          term_handles.state_reg = quad->getResidual<StateRes>();
        }
      }
    }

    // Commit.
    problem_ = std::move(problem_new);
    stage_handles_ = std::move(fresh_handles);
    terminal_handles_ = term_handles;
    horizon_length_ = H;
    dt_ = cfg.dt;
    nq_ = nq;
    nv_ = nv;
    nu_ = nu;
    stage_active_contacts_ = std::move(stage_active);
    actuation_matrix_ = std::move(actuation);

    return OCPBuildError::kNoError;
  } catch (const std::exception &) {
    return OCPBuildError::kAligatorInstantiationFailure;
  } catch (...) {
    return OCPBuildError::kAligatorInstantiationFailure;
  }
}

OCPBuildError
KinoDynamicsOCP::UpdateReferences(const PhaseContext &ctx) noexcept {
  if (!Built()) {
    return OCPBuildError::kInvalidPhaseContext;
  }
  if (ctx.ocp_type != "kinodynamics") {
    return OCPBuildError::kInvalidPhaseContext;
  }
  const auto &cfg = ctx.cost_config;
  // Topology checks — any mismatch means caller must Build instead.
  if (cfg.horizon_length != horizon_length_ || cfg.dt != dt_) {
    return OCPBuildError::kInvalidPhaseContext;
  }
  if (cfg.q_posture_ref.size() != nq_) {
    return OCPBuildError::kInvalidCostConfig;
  }
  // Per-stage active-contact check.
  bool overlap = false;
  for (int k = 0; k < horizon_length_; ++k) {
    const double t_k = static_cast<double>(k) * dt_;
    auto active = ActiveContactsAtTime(ctx.contact_plan, t_k, &overlap);
    if (overlap) {
      return OCPBuildError::kOverlappingContactPhases;
    }
    if (active != stage_active_contacts_[static_cast<std::size_t>(k)]) {
      return OCPBuildError::kInvalidPhaseContext;
    }
  }

  // Apply reference mutations through cached handles (no allocation).
  for (int k = 0; k < horizon_length_; ++k) {
    ApplyReferences(stage_handles_[static_cast<std::size_t>(k)], cfg,
                    ctx.ee_target, nq_, nv_);
  }
  ApplyReferences(terminal_handles_, cfg, ctx.ee_target, nq_, nv_);

  return OCPBuildError::kNoError;
}

} // namespace rtc::mpc
