/// @file contact_light_mpc.cpp
/// @brief Thin delegation to `MPCHandlerCore`.

#include "rtc_mpc/handler/contact_light_mpc.hpp"

#include "handler/internal/mpc_handler_core.hpp"

namespace rtc::mpc {

ContactLightMPC::ContactLightMPC() : core_(std::make_unique<internal::MPCHandlerCore>()) {}

ContactLightMPC::~ContactLightMPC() = default;

MPCInitError ContactLightMPC::Init(const MPCSolverConfig& solver_cfg,
                                   const RobotModelHandler& model, const OCPLimits& limits,
                                   const PhaseContext& initial_ctx) noexcept {
  return core_->Init(solver_cfg, model, limits, initial_ctx, ocp_);
}

MPCSolveError ContactLightMPC::Solve(const PhaseContext& ctx, const MPCStateSnapshot& state,
                                     MPCSolution& out) noexcept {
  return core_->Solve(ctx, state, ocp_, out);
}

bool ContactLightMPC::Initialised() const noexcept {
  return core_->Initialised();
}

int ContactLightMPC::horizon_length() const noexcept {
  return core_->horizon_length();
}

int ContactLightMPC::nq() const noexcept {
  return core_->nq();
}

int ContactLightMPC::nv() const noexcept {
  return core_->nv();
}

int ContactLightMPC::nu() const noexcept {
  return core_->nu();
}

int ContactLightMPC::n_contact_vars() const noexcept {
  return core_->n_contact_vars();
}

void ContactLightMPC::SeedWarmStart(const MPCSolution& prev) noexcept {
  core_->SeedWarmStart(prev, ocp_);
}

}  // namespace rtc::mpc
