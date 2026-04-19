/// @file mpc_factory.cpp
/// @brief YAML dispatch for `MPCHandlerBase` construction.

#include "rtc_mpc/handler/mpc_factory.hpp"

#include "rtc_mpc/handler/contact_rich_mpc.hpp"
#include "rtc_mpc/handler/light_contact_mpc.hpp"

#include <string>

namespace rtc::mpc {

namespace {

/// Accept either a root with `mpc:` nested or the nested map directly.
/// Returns a Node pointing at the effective MPC subtree. If neither form
/// matches, returns the input unchanged and lets downstream parsing
/// surface the missing-key failure.
[[nodiscard]] YAML::Node ResolveMpcSubtree(const YAML::Node &cfg) noexcept {
  if (!cfg.IsDefined() || !cfg.IsMap())
    return cfg;
  if (cfg["mpc"] && cfg["mpc"].IsMap())
    return cfg["mpc"];
  return cfg;
}

[[nodiscard]] bool ParseSolverConfig(const YAML::Node &solver_node,
                                     MPCSolverConfig &out) noexcept {
  if (!solver_node.IsDefined())
    return true; // leave defaults
  if (!solver_node.IsMap())
    return false;
  try {
    if (solver_node["prim_tol"])
      out.prim_tol = solver_node["prim_tol"].as<double>();
    if (solver_node["dual_tol"])
      out.dual_tol = solver_node["dual_tol"].as<double>();
    if (solver_node["mu_init"])
      out.mu_init = solver_node["mu_init"].as<double>();
    if (solver_node["max_iters"])
      out.max_iters = solver_node["max_iters"].as<int>();
    if (solver_node["max_al_iters"])
      out.max_al_iters = solver_node["max_al_iters"].as<int>();
    if (solver_node["verbose"])
      out.verbose = solver_node["verbose"].as<bool>();
  } catch (...) {
    return false;
  }
  return true;
}

[[nodiscard]] bool ParseLimits(const YAML::Node &limits_node, int nu,
                               OCPLimits &out) noexcept {
  if (!limits_node.IsDefined())
    return true;
  if (!limits_node.IsMap())
    return false;
  try {
    if (limits_node["friction_mu"])
      out.friction_mu = limits_node["friction_mu"].as<double>();
    if (limits_node["n_friction_facets"])
      out.n_friction_facets = limits_node["n_friction_facets"].as<int>();
    if (limits_node["u_min"] && limits_node["u_min"].IsSequence()) {
      const auto raw = limits_node["u_min"].as<std::vector<double>>();
      if (!raw.empty()) {
        if (static_cast<int>(raw.size()) != nu)
          return false;
        out.u_min = Eigen::Map<const Eigen::VectorXd>(
            raw.data(), static_cast<Eigen::Index>(raw.size()));
      }
    }
    if (limits_node["u_max"] && limits_node["u_max"].IsSequence()) {
      const auto raw = limits_node["u_max"].as<std::vector<double>>();
      if (!raw.empty()) {
        if (static_cast<int>(raw.size()) != nu)
          return false;
        out.u_max = Eigen::Map<const Eigen::VectorXd>(
            raw.data(), static_cast<Eigen::Index>(raw.size()));
      }
    }
  } catch (...) {
    return false;
  }
  return true;
}

} // namespace

MPCFactoryStatus
MPCFactory::Create(const YAML::Node &cfg, const RobotModelHandler &model,
                   const PhaseContext &initial_ctx,
                   std::unique_ptr<MPCHandlerBase> &handler_out) noexcept {
  handler_out.reset();
  MPCFactoryStatus status{};

  const YAML::Node mpc = ResolveMpcSubtree(cfg);
  if (!mpc.IsDefined() || !mpc.IsMap() || !mpc["ocp_type"]) {
    status.error = MPCFactoryError::kMissingOcpTypeKey;
    return status;
  }

  std::string yaml_ocp_type;
  try {
    yaml_ocp_type = mpc["ocp_type"].as<std::string>();
  } catch (...) {
    status.error = MPCFactoryError::kMissingOcpTypeKey;
    return status;
  }

  // Guard against drift between phase-manager-provided ctx.ocp_type and
  // the YAML-declared type — mismatches almost always indicate a config
  // bug rather than an intentional cross-mode bootstrap.
  if (yaml_ocp_type != initial_ctx.ocp_type) {
    status.error = MPCFactoryError::kUnknownOcpType;
    return status;
  }

  MPCSolverConfig solver_cfg{};
  if (!ParseSolverConfig(mpc["solver"], solver_cfg)) {
    status.error = MPCFactoryError::kInvalidSolverConfig;
    return status;
  }

  OCPLimits limits{};
  if (!ParseLimits(mpc["limits"], model.nu(), limits)) {
    status.error = MPCFactoryError::kInvalidLimits;
    return status;
  }

  std::unique_ptr<MPCHandlerBase> handler;
  if (yaml_ocp_type == "light_contact") {
    handler = std::make_unique<LightContactMPC>();
  } else if (yaml_ocp_type == "contact_rich") {
    handler = std::make_unique<ContactRichMPC>();
  } else {
    status.error = MPCFactoryError::kUnknownOcpType;
    return status;
  }

  const MPCInitError ie = handler->Init(solver_cfg, model, limits, initial_ctx);
  if (ie != MPCInitError::kNoError) {
    status.error = MPCFactoryError::kHandlerInitFailed;
    status.init_error = ie;
    return status;
  }

  handler_out = std::move(handler);
  return status; // kNoError
}

} // namespace rtc::mpc
