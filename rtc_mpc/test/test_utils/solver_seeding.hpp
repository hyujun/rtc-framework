#pragma once

/// @file solver_seeding.hpp
/// @brief Test-only helpers for seeding an Aligator solver with a
///        well-posed initial guess.
///
/// Phase 4 `ContactRichOCP` tests rely on `SeedGravityCompensation` to
/// avoid the NaN-on-cold-solve pathology captured as Risk #14 (see
/// `docs/mpc_implementation_progress.md` §"Phase 4 Spike Notes Q7").
/// Phase 5 `MPCHandler::SeedInitialGuess()` will promote the same pattern
/// to production. Not installed — header is test-local by design.

#include <cassert>
#include <vector>

#include <Eigen/Core>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

namespace rtc::mpc::test_utils {

/// Seed a control trajectory with robot self-gravity compensation.
/// Used by ContactRichOCP tests to avoid the NaN-on-cold-solve pathology
/// documented as Risk #14 (see docs/mpc_implementation_progress.md
/// §Phase 4 Spike Notes Q7).
///
/// Computes tau_g = pinocchio::computeGeneralizedGravity(model, data, q)
/// and broadcasts it to every control in `us`. This drives the
/// constraint-force equilibrium λ → 0 for free-fingertip scenarios
/// (no grasped object), which is the Phase 4 fixture setup.
///
/// For object-loaded grasp scenarios (Phase 5+ CLOSURE/HOLD), this
/// serves as a cold-start bootstrap only; steady-state behavior
/// requires warm-start from the previous tick's solution.
inline void SeedGravityCompensation(const pinocchio::Model &model,
                                    const Eigen::VectorXd &q,
                                    std::vector<Eigen::VectorXd> &us) noexcept {
  assert(q.size() == model.nq);
  assert(!us.empty());
  for (const auto &u : us) {
    assert(u.size() == model.nv);
    (void)u;
  }

  pinocchio::Data data(model);
  const Eigen::VectorXd tau_g =
      pinocchio::computeGeneralizedGravity(model, data, q);
  for (auto &u : us) {
    u = tau_g;
  }
}

} // namespace rtc::mpc::test_utils
