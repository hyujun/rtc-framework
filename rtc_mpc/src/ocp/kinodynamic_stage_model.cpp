/// @file kinodynamic_stage_model.cpp
/// @brief The kinodynamic running stage whose contact residuals share one
///        evaluation of the ODE and the frame kinematics (see the header).

#include "ocp/internal/kinodynamic_stage_model.hpp"

#include "rtc_mpc/ocp/contact_constraints.hpp"
#include "rtc_mpc/ocp/contact_tracking_residual.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <aligator/core/explicit-dynamics.hpp>
#include <aligator/modelling/costs/composite-costs.hpp>
#include <aligator/modelling/costs/quad-residual-cost.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <aligator/modelling/dynamics/integrator-explicit.hpp>
#pragma GCC diagnostic pop

#include <stdexcept>
#include <utility>

namespace rtc::mpc::internal {

namespace {

using Ode = KinodynamicDynamics::Ode;
using IntegratorData = aligator::dynamics::ExplicitIntegratorDataTpl<double>;
using CostStack = aligator::CostStackTpl<double>;
using CostStackData = aligator::CostStackDataTpl<double>;
using QuadResidualCost = aligator::QuadraticResidualCostTpl<double>;
using CompositeCostData = aligator::CompositeCostDataTpl<double>;

const Ode& StageOdeOrThrow(const aligator::StageModelTpl<double>& stage) {
  const Ode* ode = KinodynamicDynamics::OdeOf(stage);
  if (ode == nullptr) {
    throw std::logic_error("KinodynamicStageData: the stage has no kinodynamic integrator");
  }
  return *ode;
}

std::shared_ptr<IntegratorData::ODEData> ContinuousDataOrThrow(
    const std::shared_ptr<aligator::StageDataTpl<double>::DynamicsData>& data) {
  const auto* d = dynamic_cast<const IntegratorData*>(data.get());
  if (d == nullptr || d->continuous_data == nullptr) {
    throw std::logic_error("KinodynamicStageData: the dynamics data has no continuous data");
  }
  return d->continuous_data;
}

}  // namespace

KinodynamicStageData::KinodynamicStageData(const KinodynamicStageModel& stage)
    : Base(stage), ode(&StageOdeOrThrow(stage)), kin(*ode, ContinuousDataOrThrow(dynamics_data)) {
  // The equality constraint.
  const std::size_t eq = stage.equality_index();
  if (eq < constraint_data.size() && eq < stage.numConstraints()) {
    const auto* f = dynamic_cast<const ContactConstraintResidual*>(&*stage.constraints_.funcs[eq]);
    auto* fd = dynamic_cast<ContactConstraintData*>(constraint_data[eq].get());
    if (f != nullptr && fd != nullptr && f->bound_ode() == ode) {
      fd->shared_kin = &kin;
      shared = true;
    }
  }
  // The tracking cost's residual.
  const auto* stack = stage.getCost<CostStack>();
  auto* stack_data = dynamic_cast<CostStackData*>(cost_data.get());
  if (stack != nullptr && stack_data != nullptr) {
    const CostStack::CostKey key(stage.tracking_key());
    const auto item = stack->components_.find(key);
    const auto item_data = stack_data->sub_cost_data.find(key);
    if (item != stack->components_.end() && item_data != stack_data->sub_cost_data.end()) {
      const auto* cost = dynamic_cast<const QuadResidualCost*>(&*item->second.first);
      const auto* cd = dynamic_cast<CompositeCostData*>(item_data->second.get());
      const ContactTrackingResidual* r =
          cost != nullptr ? cost->getResidual<ContactTrackingResidual>() : nullptr;
      auto* rd =
          cd != nullptr ? dynamic_cast<ContactTrackingData*>(cd->residual_data.get()) : nullptr;
      if (r != nullptr && rd != nullptr && r->bound_ode() == ode) {
        rd->shared_kin = &kin;
        shared = true;
      }
    }
  }
}

KinodynamicStageModel::KinodynamicStageModel(const PolyCost& cost, const PolyDynamics& dynamics,
                                             std::size_t equality, std::string tracking_key)
    : Base(cost, dynamics), equality_(equality), tracking_key_(std::move(tracking_key)) {
  if (KinodynamicDynamics::OdeOf(static_cast<const Base&>(*this)) == nullptr) {
    throw std::invalid_argument(
        "KinodynamicStageModel: the dynamics is not an integrator of the kinodynamic ODE");
  }
}

void KinodynamicStageModel::evaluate(const ConstVectorRef& x, const ConstVectorRef& u,
                                     Data& data) const {
  auto* d = dynamic_cast<KinodynamicStageData*>(&data);
  if (d == nullptr || !d->shared) {
    Base::evaluate(x, u, data);
    return;
  }
  dynamics_->forward(x, u, *d->dynamics_data);
  // The continuous data now holds the ODE at (x, u): v̇ for the kinematics.
  d->kin.ForwardKinematics(d->ode->pin_model_, x);
  for (std::size_t j = 0; j < numConstraints(); ++j) {
    constraints_.funcs[j]->evaluate(x, u, *d->constraint_data[j]);
  }
  cost_->evaluate(x, u, *d->cost_data);
}

void KinodynamicStageModel::computeFirstOrderDerivatives(const ConstVectorRef& x,
                                                         const ConstVectorRef& u,
                                                         Data& data) const {
  auto* d = dynamic_cast<KinodynamicStageData*>(&data);
  if (d == nullptr || !d->shared) {
    Base::computeFirstOrderDerivatives(x, u, data);
    return;
  }
  dynamics_->dForward(x, u, *d->dynamics_data);
  // The continuous data now holds the ODE derivatives at (x, u), which
  // FrameJacobian reads; the kinematics derivatives use the v̇ of evaluate.
  d->kin.KinematicsDerivatives(d->ode->pin_model_, x);
  for (std::size_t j = 0; j < numConstraints(); ++j) {
    constraints_.funcs[j]->computeJacobians(x, u, *d->constraint_data[j]);
  }
  cost_->computeGradients(x, u, *d->cost_data);
}

std::shared_ptr<KinodynamicStageModel::Data> KinodynamicStageModel::createData() const {
  return std::make_shared<KinodynamicStageData>(*this);
}

}  // namespace rtc::mpc::internal
