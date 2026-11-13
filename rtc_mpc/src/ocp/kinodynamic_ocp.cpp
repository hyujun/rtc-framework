#include "rtc_mpc/ocp/kinodynamic_ocp.hpp"

#include "rtc_mpc/reference/stage_reference.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <aligator/core/stage-model.hpp>
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/modelling/constraints/equality-constraint.hpp>
#include <aligator/modelling/constraints/negative-orthant.hpp>
#include <aligator/modelling/costs/quad-residual-cost.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#pragma GCC diagnostic pop

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace rtc::mpc {

namespace {

using CostStack = aligator::CostStackTpl<double>;
using QuadResidualCost = aligator::QuadraticResidualCostTpl<double>;
using StageModel = aligator::StageModelTpl<double>;
using StageFunction = aligator::StageFunctionTpl<double>;
using EqualitySet = aligator::EqualityConstraintTpl<double>;
using NegativeOrthant = aligator::NegativeOrthantTpl<double>;

[[nodiscard]] bool AllFinite(const double* p, int n) noexcept {
  for (int i = 0; i < n; ++i) {
    if (!std::isfinite(p[i])) {
      return false;
    }
  }
  return true;
}

}  // namespace

KinodynamicOCP::KinodynamicOCP(KinodynamicOCPConfig cfg) : cfg_(std::move(cfg)) {}

OCPBuildError KinodynamicOCP::Build(const PhaseContext& ctx, const RobotModelHandler& model,
                                    const OCPLimits& limits) noexcept {
  // Every check first: a refused Build leaves a built problem as it was, so the
  // handler core's rebuild on kInvalidPhaseContext — which cannot change what
  // this OCP's config fixes — neither destroys the problem nor allocates.
  if (!model.Initialised() || !model.floating_base()) {
    return OCPBuildError::kModelNotInitialised;
  }
  if (ctx.ocp_type != kType) {
    return OCPBuildError::kInvalidPhaseContext;
  }
  if (ctx.cost_config.horizon_length != cfg_.horizon_nodes || ctx.cost_config.dt != cfg_.dt ||
      cfg_.state_weights.size() != 2 * model.nv() ||
      cfg_.terminal_weights.size() != 2 * model.nv() || cfg_.input_weights.size() != model.nu()) {
    return OCPBuildError::kInvalidCostConfig;
  }
  // No input box in this formulation (hard limits are the whole-body QP's):
  // one is refused, not dropped.
  if (limits.u_min.size() != 0 || limits.u_max.size() != 0) {
    return OCPBuildError::kLimitsDimMismatch;
  }
  if (ctx.horizon != nullptr) {
    const OCPBuildError e = CheckContext(ctx, model);
    if (e != OCPBuildError::kNoError) {
      return e;
    }
  }

  problem_.reset();
  stages_.clear();
  terminal_ = nullptr;
  model_ = nullptr;
  eq_data_.reset();
  ineq_data_.reset();
  if (dyn_.Init(model) != KinodynamicError::kNoError) {
    return OCPBuildError::kModelNotInitialised;
  }
  const int nv = model.nv();
  const int nu = dyn_.nu();
  const int nc = dyn_.n_contacts();

  try {
    const KinodynamicDynamics::Space& space = *dyn_.space();
    const Eigen::VectorXd x0 = space.neutral();
    const Eigen::MatrixXd q_w = cfg_.state_weights.asDiagonal();
    const Eigen::MatrixXd r_w = cfg_.input_weights.asDiagonal();
    const Eigen::MatrixXd qf_w = (cfg_.terminal_scaling * cfg_.terminal_weights).asDiagonal();
    Eigen::VectorXd w_track(kContactTrackingRows * nc);
    for (int c = 0; c < nc; ++c) {
      w_track.segment<kContactTrackingRows>(kContactTrackingRows * c) =
          cfg_.contact_tracking_weights;
    }
    const Eigen::MatrixXd track_w = w_track.asDiagonal();

    // Full stance until a horizon says otherwise.
    const std::uint32_t stance = dyn_.n_modes() - 1U;
    const std::optional<KinodynamicDynamics::Ode> ode = dyn_.MakeOde(stance);
    if (!ode.has_value()) {
      return OCPBuildError::kAligatorInstantiationFailure;
    }
    // Unbound here; each stage's copy is bound to that stage's ODE below.
    const KinodynamicConstraintsConfig& cc = cfg_.constraints;
    const ContactTrackingResidual tracking(*ode);
    const ContactConstraintResidual equality(*ode, cc.foot_constraint);
    const ContactWrenchConeResidual inequality(*ode, cc.friction, cc.contact_rectangle);

    // The running cost is integrated over the node interval (× dt).
    CostStack cost(space, nu);
    cost.addCost(std::string(kStateCostKey), QuadStateCost(space, nu, x0, q_w), cfg_.dt);
    cost.addCost(std::string(kInputCostKey), QuadControlCost(space, Eigen::VectorXd::Zero(nu), r_w),
                 cfg_.dt);
    cost.addCost(std::string(kContactTrackingCostKey),
                 QuadResidualCost(space, xyz::polymorphic<StageFunction>(tracking), track_w),
                 cfg_.dt);
    cost.addCost(std::string(kJointLimitCostKey),
                 JointLimitBarrierCost(space, nu, model.model(), cc.joint_limits), cfg_.dt);
    if (cc.collision_enabled) {
      cost.addCost(std::string(kCollisionCostKey),
                   CollisionBarrierCost(space, *ode, cc.collision_pairs, cc.collision), cfg_.dt);
    }

    std::vector<xyz::polymorphic<StageModel>> stages;
    stages.reserve(static_cast<std::size_t>(cfg_.horizon_nodes));
    for (int k = 0; k < cfg_.horizon_nodes; ++k) {
      std::optional<KinodynamicDynamics::StageDynamics> dyn;
      if (dyn_.MakeStageDynamics(stance, cfg_.dt, cfg_.integrator, dyn) !=
          KinodynamicError::kNoError) {
        return OCPBuildError::kInvalidCostConfig;
      }
      StageModel stage(cost, *dyn);
      stage.addConstraint(xyz::polymorphic<StageFunction>(equality), EqualitySet());
      stage.addConstraint(xyz::polymorphic<StageFunction>(inequality), NegativeOrthant());
      stages.emplace_back(std::move(stage));
    }
    problem_.emplace(x0, stages, QuadStateCost(space, nu, x0, qf_w));

    // Resolve, in the problem's own copies, what the ticks write.
    stages_.resize(static_cast<std::size_t>(cfg_.horizon_nodes));
    for (std::size_t k = 0; k < stages_.size(); ++k) {
      StageModel& stage = *problem_->stages_[k];
      StageHandles& h = stages_[k];
      h.ode = KinodynamicDynamics::OdeOf(stage);
      auto* stack = stage.getCost<CostStack>();
      if (h.ode == nullptr || stack == nullptr) {
        throw std::logic_error("stage layout");
      }
      h.state = stack->getComponent<QuadStateCost>(std::string(kStateCostKey));
      h.input = stack->getComponent<QuadControlCost>(std::string(kInputCostKey));
      auto* track = stack->getComponent<QuadResidualCost>(std::string(kContactTrackingCostKey));
      h.tracking = track != nullptr ? track->getResidual<ContactTrackingResidual>() : nullptr;
      if (h.state == nullptr || h.input == nullptr || h.tracking == nullptr ||
          !h.tracking->Bind(h.ode)) {
        throw std::logic_error("stage layout");
      }
      h.equality = stage.constraints_.getConstraint<ContactConstraintResidual>(kEqualityConstraint);
      h.inequality =
          stage.constraints_.getConstraint<ContactWrenchConeResidual>(kInequalityConstraint);
      if (h.equality == nullptr || h.inequality == nullptr || !h.equality->Bind(h.ode) ||
          !h.inequality->Bind(h.ode)) {
        throw std::logic_error("constraint layout");
      }
      if (cc.collision_enabled) {
        h.collision = stack->getComponent<CollisionBarrierCost>(std::string(kCollisionCostKey));
        if (h.collision == nullptr || !h.collision->Bind(h.ode)) {
          throw std::logic_error("collision layout");
        }
      }
    }
    terminal_ = dynamic_cast<QuadStateCost*>(&*problem_->term_cost_);
    if (terminal_ == nullptr) {
      throw std::logic_error("terminal layout");
    }
    eq_data_ = stages_.front().equality->createData();
    ineq_data_ = stages_.front().inequality->createData();
  } catch (...) {
    problem_.reset();
    stages_.clear();
    terminal_ = nullptr;
    eq_data_.reset();
    ineq_data_.reset();
    return OCPBuildError::kAligatorInstantiationFailure;
  }

  model_ = &model;
  nq_ = model.nq();
  nv_ = nv;
  nu_ = nu;
  n_contact_vars_ = model.layout().n_contact_vars;
  x_buf_ = Eigen::VectorXd::Zero(nq_ + nv_);
  u_buf_ = Eigen::VectorXd::Zero(nu_);

  if (ctx.horizon != nullptr) {
    WriteHorizon(*ctx.horizon);  // checked above
  }
  return OCPBuildError::kNoError;
}

OCPBuildError KinodynamicOCP::CheckContext(const PhaseContext& ctx,
                                           const RobotModelHandler& model) const noexcept {
  if (ctx.ocp_type != kType || ctx.cost_config.horizon_length != cfg_.horizon_nodes ||
      ctx.cost_config.dt != cfg_.dt) {
    return OCPBuildError::kInvalidPhaseContext;
  }
  if (ctx.horizon == nullptr) {
    return OCPBuildError::kMissingHorizonReference;
  }
  const HorizonReference& h = *ctx.horizon;
  const int nq = model.nq();
  const int nv = model.nv();
  const int nc = model.n_contacts();
  const int ncv = model.layout().n_contact_vars;
  if (h.n_stages != static_cast<std::uint32_t>(cfg_.horizon_nodes + 1) ||
      h.n_contacts != static_cast<std::uint32_t>(nc) || h.nq != static_cast<std::uint32_t>(nq) ||
      h.nv != static_cast<std::uint32_t>(nv) ||
      h.n_contact_vars != static_cast<std::uint32_t>(ncv) || h.dt != cfg_.dt) {
    return OCPBuildError::kInvalidPhaseContext;
  }
  const std::uint32_t n_modes = 1U << static_cast<unsigned>(nc);
  for (int k = 0; k <= cfg_.horizon_nodes; ++k) {
    const StageReference& s = h.stages[static_cast<std::size_t>(k)];
    if (!AllFinite(s.q.data(), nq) || !AllFinite(s.v.data(), nv) ||
        !model.layout().BaseQuaternionIsUnit(Eigen::Map<const Eigen::VectorXd>(s.q.data(), nq))) {
      return OCPBuildError::kInvalidHorizonReference;
    }
    if (k == cfg_.horizon_nodes) {
      break;  // the terminal node carries a state only
    }
    // The mode SetMode takes, the wrench u_ref takes, and what SetFromStage
    // takes (its own check).
    if (s.mode >= n_modes || !AllFinite(s.wrench.data(), ncv) ||
        !ContactTrackingResidual::AcceptsStage(s, h.terrain, nc) ||
        !ContactConstraintResidual::AcceptsStage(s, h.terrain, nc)) {
      return OCPBuildError::kInvalidHorizonReference;
    }
  }
  return OCPBuildError::kNoError;
}

void KinodynamicOCP::WriteStateTarget(const StageReference& s, QuadStateCost& cost) noexcept {
  x_buf_.head(nq_) = Eigen::Map<const Eigen::VectorXd>(s.q.data(), nq_);
  x_buf_.tail(nv_) = Eigen::Map<const Eigen::VectorXd>(s.v.data(), nv_);
  cost.setTarget(x_buf_);
}

void KinodynamicOCP::WriteHorizon(const HorizonReference& h) noexcept {
  for (std::size_t k = 0; k < stages_.size(); ++k) {
    const StageReference& s = h.stages[k];
    StageHandles& st = stages_[k];
    // CheckContext ran every test these make, so none of them refuses.
    static_cast<void>(dyn_.SetMode(*st.ode, s.mode));
    WriteStateTarget(s, *st.state);
    u_buf_.head(n_contact_vars_) =
        Eigen::Map<const Eigen::VectorXd>(s.wrench.data(), n_contact_vars_);
    st.input->setTarget(u_buf_);
    static_cast<void>(st.tracking->SetFromStage(s, h.terrain));
    static_cast<void>(st.equality->SetFromStage(s, h.terrain));
  }
  WriteStateTarget(h.stages[stages_.size()], *terminal_);
}

OCPBuildError KinodynamicOCP::UpdateReferences(const PhaseContext& ctx) noexcept {
  if (!Built()) {
    return OCPBuildError::kInvalidPhaseContext;
  }
  const OCPBuildError e = CheckContext(ctx, *model_);
  if (e != OCPBuildError::kNoError) {
    return e;
  }
  WriteHorizon(*ctx.horizon);
  return OCPBuildError::kNoError;
}

bool KinodynamicOCP::ReferenceControl(int k, Eigen::Ref<Eigen::VectorXd> u) const noexcept {
  if (!Built() || k < 0 || k >= static_cast<int>(stages_.size()) || u.size() != nu_) {
    return false;
  }
  u = stages_[static_cast<std::size_t>(k)].input->getTarget();
  return true;
}

bool KinodynamicOCP::ConstraintResiduals(const std::vector<Eigen::VectorXd>& xs,
                                         const std::vector<Eigen::VectorXd>& us,
                                         ConstraintResidualReport& out) noexcept {
  if (!Built() || xs.size() != stages_.size() + 1 || us.size() != stages_.size() ||
      xs.back().size() != nq_ + nv_ || !xs.back().allFinite()) {
    return false;
  }
  for (std::size_t k = 0; k < stages_.size(); ++k) {
    // A non-finite trajectory has no residual: max / min would skip its NaNs
    // and report it as clean.
    if (xs[k].size() != nq_ + nv_ || us[k].size() != nu_ || !xs[k].allFinite() ||
        !us[k].allFinite()) {
      return false;
    }
  }
  constexpr double kInf = std::numeric_limits<double>::infinity();
  ConstraintResidualReport r;
  r.min_friction_margin.fill(kInf);
  r.min_cop_margin.fill(kInf);
  auto& eq = static_cast<ContactConstraintData&>(*eq_data_);
  auto& ineq = static_cast<ContactWrenchConeData&>(*ineq_data_);
  try {
    for (std::size_t k = 0; k < stages_.size(); ++k) {
      const StageHandles& st = stages_[k];
      st.equality->evaluate(xs[k], us[k], eq);
      st.inequality->evaluate(xs[k], us[k], ineq);
      if (!eq.value_.allFinite() || !ineq.value_.allFinite()) {
        return false;
      }
      const int eq_rows = st.equality->rows_per_contact();
      const int in_rows = st.inequality->rows_per_contact();
      const int nc = st.equality->n_contacts();
      for (int c = 0; c < nc; ++c) {
        const auto ci = static_cast<std::size_t>(c);
        const Eigen::Index eo = static_cast<Eigen::Index>(c) * eq_rows;
        if (eq.stance[ci] == 0) {
          r.swing_equality_max =
              std::max(r.swing_equality_max, std::fabs(eq.value_[eo + eq_rows - 1]));
          continue;
        }
        r.stance_equality_max = std::max(r.stance_equality_max,
                                         eq.value_.segment(eo, eq_rows - 1).cwiseAbs().maxCoeff());
        const Eigen::Index io = static_cast<Eigen::Index>(c) * in_rows;
        r.inequality_violation_max =
            std::max(r.inequality_violation_max, ineq.value_.segment(io, in_rows).maxCoeff());
        r.min_friction_margin[ci] = std::min(r.min_friction_margin[ci], -ineq.value_[io]);
        if (in_rows > 1) {
          // CoP rows are −(distance · F_z) with F_z on the frame's own axes.
          const Eigen::Index w = static_cast<Eigen::Index>(c) * st.inequality->force_size();
          const Eigen::Matrix3d& rot = ineq.pin_data.oMf[st.ode->contact_ids_[ci]].rotation();
          const double fz = rot.col(2).dot(Eigen::Vector3d(us[k].segment<3>(w)));
          const double worst = ineq.value_.segment(io + 1, in_rows - 1).maxCoeff();
          const double margin = fz > 0.0 ? -worst / fz : -kInf;
          r.min_cop_margin[ci] = std::min(r.min_cop_margin[ci], margin);
        }
      }
    }
  } catch (...) {
    return false;
  }
  out = r;
  return true;
}

}  // namespace rtc::mpc
