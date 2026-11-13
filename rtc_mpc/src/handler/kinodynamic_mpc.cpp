/// @file kinodynamic_mpc.cpp
/// @brief Thin delegation to `MPCHandlerCore`.

#include "rtc_mpc/handler/kinodynamic_mpc.hpp"

#include "handler/internal/mpc_handler_core.hpp"

#include <utility>

namespace rtc::mpc {

KinodynamicMPC::KinodynamicMPC(KinodynamicOCPConfig cfg)
    : ocp_(std::move(cfg)), core_(std::make_unique<internal::MPCHandlerCore>()) {}

KinodynamicMPC::~KinodynamicMPC() = default;

MPCInitError KinodynamicMPC::Init(const MPCSolverConfig& solver_cfg, const RobotModelHandler& model,
                                  const OCPLimits& limits,
                                  const PhaseContext& initial_ctx) noexcept {
  return core_->Init(solver_cfg, model, limits, initial_ctx, ocp_);
}

MPCSolveError KinodynamicMPC::Solve(const PhaseContext& ctx, const MPCStateSnapshot& state,
                                    MPCSolution& out) noexcept {
  return core_->Solve(ctx, state, ocp_, out);
}

bool KinodynamicMPC::Initialised() const noexcept {
  return core_->Initialised();
}

const MPCSolverConfig& KinodynamicMPC::solver_config() const noexcept {
  return core_->solver_config();
}

bool KinodynamicMPC::InitialSolvePending() const noexcept {
  return core_->InitialSolvePending();
}

int KinodynamicMPC::horizon_length() const noexcept {
  return core_->horizon_length();
}

int KinodynamicMPC::nq() const noexcept {
  return core_->nq();
}

int KinodynamicMPC::nv() const noexcept {
  return core_->nv();
}

int KinodynamicMPC::nu() const noexcept {
  return core_->nu();
}

int KinodynamicMPC::n_contact_vars() const noexcept {
  return core_->n_contact_vars();
}

void KinodynamicMPC::SeedWarmStart(const MPCSolution& prev) noexcept {
  core_->SeedWarmStart(prev, ocp_);
}

}  // namespace rtc::mpc
