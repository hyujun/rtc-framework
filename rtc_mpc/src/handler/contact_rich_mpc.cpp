/// @file contact_rich_mpc.cpp
/// @brief Thin delegation to `MPCHandlerCore`.

#include "rtc_mpc/handler/contact_rich_mpc.hpp"

#include "handler/internal/mpc_handler_core.hpp"

namespace rtc::mpc {

ContactRichMPC::ContactRichMPC()
    : core_(std::make_unique<internal::MPCHandlerCore>()) {}

ContactRichMPC::~ContactRichMPC() = default;

MPCInitError ContactRichMPC::Init(const MPCSolverConfig &solver_cfg,
                                  const RobotModelHandler &model,
                                  const OCPLimits &limits,
                                  const PhaseContext &initial_ctx) noexcept {
  return core_->Init(solver_cfg, model, limits, initial_ctx, ocp_);
}

MPCSolveError ContactRichMPC::Solve(const PhaseContext &ctx,
                                    const MPCStateSnapshot &state,
                                    MPCSolution &out) noexcept {
  return core_->Solve(ctx, state, ocp_, out);
}

bool ContactRichMPC::Initialised() const noexcept {
  return core_->Initialised();
}

int ContactRichMPC::horizon_length() const noexcept {
  return core_->horizon_length();
}

int ContactRichMPC::nq() const noexcept { return core_->nq(); }
int ContactRichMPC::nv() const noexcept { return core_->nv(); }
int ContactRichMPC::nu() const noexcept { return core_->nu(); }
int ContactRichMPC::n_contact_vars() const noexcept {
  return core_->n_contact_vars();
}

void ContactRichMPC::SeedWarmStart(const MPCSolution &prev) noexcept {
  core_->SeedWarmStart(prev, ocp_);
}

} // namespace rtc::mpc
