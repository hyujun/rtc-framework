/// @file light_contact_mpc.cpp
/// @brief Thin delegation to `MPCHandlerCore`.

#include "rtc_mpc/handler/light_contact_mpc.hpp"

#include "handler/internal/mpc_handler_core.hpp"

namespace rtc::mpc {

LightContactMPC::LightContactMPC()
    : core_(std::make_unique<internal::MPCHandlerCore>()) {}

LightContactMPC::~LightContactMPC() = default;

MPCInitError LightContactMPC::Init(const MPCSolverConfig &solver_cfg,
                                   const RobotModelHandler &model,
                                   const OCPLimits &limits,
                                   const PhaseContext &initial_ctx) noexcept {
  return core_->Init(solver_cfg, model, limits, initial_ctx, ocp_);
}

MPCSolveError LightContactMPC::Solve(const PhaseContext &ctx,
                                     const MPCStateSnapshot &state,
                                     MPCSolution &out) noexcept {
  return core_->Solve(ctx, state, ocp_, out);
}

bool LightContactMPC::Initialised() const noexcept {
  return core_->Initialised();
}

int LightContactMPC::horizon_length() const noexcept {
  return core_->horizon_length();
}

int LightContactMPC::nq() const noexcept { return core_->nq(); }
int LightContactMPC::nv() const noexcept { return core_->nv(); }
int LightContactMPC::nu() const noexcept { return core_->nu(); }
int LightContactMPC::n_contact_vars() const noexcept {
  return core_->n_contact_vars();
}

void LightContactMPC::SeedWarmStart(const MPCSolution &prev) noexcept {
  core_->SeedWarmStart(prev, ocp_);
}

} // namespace rtc::mpc
