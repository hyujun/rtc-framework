#include "rtc_mpc/ocp/contact_rich_ocp.hpp"

#include "ocp/internal/constraint_models.hpp"
#include "rtc_mpc/ocp/grasp_quality_provider.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <aligator/core/stage-model.hpp>
#include <aligator/modelling/constraints/negative-orthant.hpp>
#include <aligator/modelling/costs/quad-residual-cost.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <aligator/modelling/dynamics/integrator-semi-euler.hpp>
#include <aligator/modelling/dynamics/multibody-constraint-fwd.hpp>
#include <aligator/modelling/multibody/contact-force.hpp>
#include <aligator/modelling/multibody/frame-placement.hpp>
#include <aligator/modelling/multibody/multibody-friction-cone.hpp>
#include <aligator/modelling/spaces/multibody.hpp>
#include <aligator/modelling/state-error.hpp>
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
using ContactForceRes = aligator::ContactForceResidualTpl<double>;
using FrictionConeRes = aligator::MultibodyFrictionConeResidualTpl<double>;
using NegOrthant = aligator::NegativeOrthantTpl<double>;
using PhaseSpace = aligator::MultibodyPhaseSpace<double>;
using MultibodyConstraintFwd =
    aligator::dynamics::MultibodyConstraintFwdDynamicsTpl<double>;
using IntegratorSE = aligator::dynamics::IntegratorSemiImplEulerTpl<double>;
using StageModel = aligator::StageModelTpl<double>;
using TrajOptProblem = aligator::TrajOptProblemTpl<double>;

// Mirrors `LightContactOCP::ActiveContactsAtTime` (local to this TU — kept
// independent so refactors here don't perturb the shipped LightContact path).
std::vector<int> ActiveContactsAtTime(const ContactPlan &plan, double t,
                                      bool *overlap_found) noexcept {
  std::vector<int> active{};
  int cover_count = 0;
  for (const auto &phase : plan.phases) {
    if (t >= phase.t_start && t < phase.t_end) {
      ++cover_count;
      if (cover_count == 1) {
        active = phase.active_frame_ids;
      } else {
        if (overlap_found != nullptr) {
          *overlap_found = true;
        }
      }
    }
  }
  return active;
}

// Walk a freshly-built StageModel's polymorphic cost tree and populate
// raw-pointer handles for alloc-free mutation in UpdateReferences. The
// `contact_force` vector is parallel to `active_fids` order.
RichStageHandles CacheStageHandles(StageModel &stage,
                                   const StageComponentKeys &keys,
                                   const std::vector<int> &active_fids,
                                   const std::vector<ContactFrameInfo> &frames,
                                   bool include_contact_force) noexcept {
  RichStageHandles h{};
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

  if (include_contact_force) {
    h.contact_force.reserve(active_fids.size());
    for (int fid : active_fids) {
      std::string name = "contact_" + std::to_string(fid);
      for (const auto &info : frames) {
        if (info.frame_id == fid) {
          name = info.name;
          break;
        }
      }
      const std::string key = std::string(kCostKeyContactForcePrefix) + name;
      auto *quad = stack->getComponent<QuadCost>(key);
      ContactForceRes *res = nullptr;
      if (quad != nullptr) {
        res = quad->getResidual<ContactForceRes>();
      }
      h.contact_force.push_back(res);
    }
  }
  return h;
}

// Mutate cached handles to reflect new references. Pre-condition: topology
// unchanged (caller verified); handles non-null only where corresponding
// components were built.
void ApplyReferences(const RichStageHandles &h, const PhaseCostConfig &cfg,
                     const pinocchio::SE3 &ee_target, int nq, int nv,
                     const Eigen::VectorXd &F_target_full,
                     const std::vector<int> &active_fids) noexcept {
  if (h.frame_placement != nullptr) {
    h.frame_placement->setReference(ee_target);
  }
  if (h.state_reg != nullptr) {
    Eigen::VectorXd x_target(nq + nv);
    x_target.head(nq) = cfg.q_posture_ref;
    x_target.tail(nv).setZero();
    h.state_reg->target_ = x_target;
  }
  (void)h.control_reg; // target = 0, no mutation needed.

  // Per-active-contact force-reference mutation. F_target_full is a flat
  // 6-vector shared across contacts (first 3 = linear, last 3 = angular);
  // residuals size themselves to match their own contact dim (3 or 6).
  for (std::size_t i = 0; i < h.contact_force.size(); ++i) {
    auto *res = h.contact_force[i];
    if (res == nullptr) {
      continue;
    }
    const int dim = static_cast<int>(res->getReference().size());
    Eigen::VectorXd f_ref(dim);
    if (dim <= F_target_full.size()) {
      f_ref = F_target_full.head(dim);
    } else {
      f_ref.setZero();
      f_ref.head(F_target_full.size()) = F_target_full;
    }
    res->setReference(f_ref);
    (void)active_fids; // reserved for future per-frame overrides.
  }
}

} // namespace

OCPBuildError ContactRichOCP::Build(const PhaseContext &ctx,
                                    const RobotModelHandler &model,
                                    const OCPLimits &limits) noexcept {
  if (!model.Initialised()) {
    return OCPBuildError::kModelNotInitialised;
  }
  if (ctx.ocp_type != "contact_rich") {
    return OCPBuildError::kInvalidPhaseContext;
  }
  const auto &cfg = ctx.cost_config;
  if (cfg.horizon_length <= 0 || cfg.dt <= 0.0) {
    return OCPBuildError::kInvalidCostConfig;
  }

  const int nq = model.nq();
  const int nv = model.nv();
  const int nu = nv;

  if (cfg.q_posture_ref.size() != nq) {
    return OCPBuildError::kInvalidCostConfig;
  }
  if (limits.u_min.size() != 0 && limits.u_min.size() != nu) {
    return OCPBuildError::kLimitsDimMismatch;
  }
  if (limits.u_max.size() != 0 && limits.u_max.size() != nu) {
    return OCPBuildError::kLimitsDimMismatch;
  }

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

  const bool include_contact_force = cfg.w_contact_force > 0.0;
  const bool include_friction_cone = limits.friction_mu > 0.0;

  try {
    PhaseSpace space(model.model());
    Eigen::MatrixXd actuation = Eigen::MatrixXd::Identity(nv, nv);
    pinocchio::ProximalSettings prox_settings(1e-9, 1e-10, 10);
    const int ndx = space.ndx();

    std::vector<xyz::polymorphic<StageModel>> stages{};
    stages.reserve(static_cast<std::size_t>(H));

    std::vector<StageComponentKeys> stage_keys{};
    stage_keys.reserve(static_cast<std::size_t>(H));

    for (int k = 0; k < H; ++k) {
      const auto &active_fids = stage_active[static_cast<std::size_t>(k)];

      // 1. Baseline cost stack.
      CostFactoryError cost_err = CostFactoryError::kNoError;
      auto stage_cost =
          cost_factory::BuildRunningCost(cfg, model, ctx.ee_target, &cost_err);
      if (cost_err != CostFactoryError::kNoError) {
        return OCPBuildError::kInvalidCostConfig;
      }

      // 2. Dynamics (rigid-contact fwd on active contacts only).
      auto constraint_models =
          internal::BuildConstraintModels(model, active_fids);
      MultibodyConstraintFwd cont_dyn(space, actuation, constraint_models,
                                      prox_settings);

      // 3. Per-active-contact ContactForceResidual (cost term).
      if (include_contact_force) {
        for (int fid : active_fids) {
          std::string name = "contact_" + std::to_string(fid);
          int dim = 3;
          for (const auto &info : model_frames) {
            if (info.frame_id == fid) {
              name = info.name;
              dim = info.dim;
              break;
            }
          }
          Eigen::VectorXd fref(dim);
          fref.setZero();
          if (cfg.F_target.size() > 0) {
            const int copy_n = std::min<int>(dim, cfg.F_target.size());
            fref.head(copy_n) = cfg.F_target.head(copy_n);
          }
          ContactForceRes cfr(ndx, model.model(), actuation, constraint_models,
                              prox_settings, fref, name);
          Eigen::MatrixXd Wcfr =
              cfg.w_contact_force * Eigen::MatrixXd::Identity(dim, dim);
          QuadCost qcost(space, cfr, Wcfr);
          stage_cost.stack.addCost(
              std::string(kCostKeyContactForcePrefix) + name, qcost, 1.0);
        }
      }

      // 4. Grasp-quality provider hook (null by default in Phase 4).
      if (grasp_quality_provider_ != nullptr) {
        const OCPBuildError prov_err =
            grasp_quality_provider_->AppendRunningCost(stage_cost, model,
                                                       active_fids);
        if (prov_err != OCPBuildError::kNoError) {
          return prov_err;
        }
      }

      // 5. Compose stage (cost + dynamics). Integrator wraps the ODE.
      IntegratorSE integrator(
          xyz::polymorphic<aligator::dynamics::ODEAbstractTpl<double>>(
              cont_dyn),
          cfg.dt);
      StageModel stage(stage_cost.stack, integrator);

      // 6. Attach friction-cone inequality constraints per active contact.
      if (include_friction_cone) {
        for (int fid : active_fids) {
          std::string name = "contact_" + std::to_string(fid);
          for (const auto &info : model_frames) {
            if (info.frame_id == fid) {
              name = info.name;
              break;
            }
          }
          FrictionConeRes fcone(ndx, model.model(), actuation,
                                constraint_models, prox_settings, name,
                                limits.friction_mu);
          stage.addConstraint(fcone, NegOrthant());
        }
      }

      stages.emplace_back(stage);
      stage_keys.push_back(stage_cost.keys);
    }

    // Terminal cost (no control reg; no contact-force cost).
    CostFactoryError term_err = CostFactoryError::kNoError;
    auto term_cost =
        cost_factory::BuildTerminalCost(cfg, model, ctx.ee_target, &term_err);
    if (term_err != CostFactoryError::kNoError) {
      return OCPBuildError::kInvalidCostConfig;
    }
    if (grasp_quality_provider_ != nullptr) {
      // Terminal stage has no active-contact variable on the u-less cost
      // branch; pass the last running-stage active set as a best-effort
      // signal. Default provider impl is a no-op.
      const auto &last_active =
          stage_active.empty() ? std::vector<int>{} : stage_active.back();
      const OCPBuildError prov_err =
          grasp_quality_provider_->AppendTerminalCost(term_cost, model,
                                                      last_active);
      if (prov_err != OCPBuildError::kNoError) {
        return prov_err;
      }
    }

    Eigen::VectorXd x0 = space.neutral();
    auto problem_new =
        std::make_unique<TrajOptProblem>(x0, stages, term_cost.stack);

    // Walk the STORED stages to cache raw handles. Pointers into `stages`
    // (local) would dangle — the TrajOptProblem ctor copied that vector
    // into `problem_new->stages_`.
    std::vector<RichStageHandles> fresh_handles{};
    fresh_handles.reserve(static_cast<std::size_t>(H));
    for (int k = 0; k < H; ++k) {
      auto &stored_stage = *problem_new->stages_[static_cast<std::size_t>(k)];
      fresh_handles.push_back(CacheStageHandles(
          stored_stage, stage_keys[static_cast<std::size_t>(k)],
          stage_active[static_cast<std::size_t>(k)], model_frames,
          include_contact_force));
    }

    // Terminal handles (frame_placement + state_reg only; no cforce).
    RichStageHandles term_handles{};
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

    // Commit state.
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
    limits_cached_ = limits;
    w_contact_force_cached_ = cfg.w_contact_force;

    return OCPBuildError::kNoError;
  } catch (const std::exception &) {
    return OCPBuildError::kAligatorInstantiationFailure;
  } catch (...) {
    return OCPBuildError::kAligatorInstantiationFailure;
  }
}

OCPBuildError
ContactRichOCP::UpdateReferences(const PhaseContext &ctx) noexcept {
  if (!Built()) {
    return OCPBuildError::kInvalidPhaseContext;
  }
  if (ctx.ocp_type != "contact_rich") {
    return OCPBuildError::kInvalidPhaseContext;
  }
  const auto &cfg = ctx.cost_config;

  // Topology checks.
  if (cfg.horizon_length != horizon_length_ || cfg.dt != dt_) {
    return OCPBuildError::kInvalidPhaseContext;
  }
  if (cfg.q_posture_ref.size() != nq_) {
    return OCPBuildError::kInvalidCostConfig;
  }
  // w_contact_force crossing 0 means handle layout changed → rebuild.
  const bool was_cforce = w_contact_force_cached_ > 0.0;
  const bool now_cforce = cfg.w_contact_force > 0.0;
  if (was_cforce != now_cforce) {
    return OCPBuildError::kInvalidPhaseContext;
  }

  // Per-stage active-contact parity check.
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

  // Prepare the per-tick F_target vector (expanded to 6-D so residuals of
  // size 3 or 6 can both take their prefix slice).
  Eigen::VectorXd F_full = Eigen::VectorXd::Zero(6);
  if (cfg.F_target.size() > 0) {
    const int copy_n = std::min<int>(6, static_cast<int>(cfg.F_target.size()));
    F_full.head(copy_n) = cfg.F_target.head(copy_n);
  }

  for (int k = 0; k < horizon_length_; ++k) {
    ApplyReferences(stage_handles_[static_cast<std::size_t>(k)], cfg,
                    ctx.ee_target, nq_, nv_, F_full,
                    stage_active_contacts_[static_cast<std::size_t>(k)]);
  }
  ApplyReferences(terminal_handles_, cfg, ctx.ee_target, nq_, nv_, F_full,
                  /*active_fids=*/{});

  if (grasp_quality_provider_ != nullptr) {
    grasp_quality_provider_->UpdateReferences(ctx);
  }

  return OCPBuildError::kNoError;
}

} // namespace rtc::mpc
