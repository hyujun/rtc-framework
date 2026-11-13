/// @file mpc_factory.cpp
/// @brief YAML dispatch for `MPCHandlerBase` construction.

#include "rtc_mpc/handler/mpc_factory.hpp"

#include "rtc_mpc/handler/contact_light_mpc.hpp"
#include "rtc_mpc/handler/contact_rich_mpc.hpp"
#include "rtc_mpc/handler/kinodynamic_mpc.hpp"

#include <string>
#include <utility>

namespace rtc::mpc {

namespace {

/// Accept either a root with `mpc:` nested or the nested map directly.
/// Returns a Node pointing at the effective MPC subtree. If neither form
/// matches, returns the input unchanged and lets downstream parsing
/// surface the missing-key failure.
[[nodiscard]] YAML::Node ResolveMpcSubtree(const YAML::Node& cfg) noexcept {
  if (!cfg.IsDefined() || !cfg.IsMap())
    return cfg;
  if (cfg["mpc"] && cfg["mpc"].IsMap())
    return cfg["mpc"];
  return cfg;
}

[[nodiscard]] bool ParseRolloutType(const std::string& name, MPCRolloutType& out) noexcept {
  if (name == "linear") {
    out = MPCRolloutType::kLinear;
  } else if (name == "nonlinear") {
    out = MPCRolloutType::kNonlinear;
  } else {
    return false;
  }
  return true;
}

[[nodiscard]] bool ParseLinearSolver(const std::string& name, MPCLinearSolver& out) noexcept {
  if (name == "serial") {
    out = MPCLinearSolver::kSerial;
  } else if (name == "parallel") {
    out = MPCLinearSolver::kParallel;
  } else {
    return false;
  }
  return true;
}

[[nodiscard]] bool ParseLimits(const YAML::Node& limits_node, int nu, OCPLimits& out) noexcept {
  if (!limits_node.IsDefined())
    return true;
  if (!limits_node.IsMap())
    return false;
  try {
    if (limits_node["friction_mu"])
      out.friction_mu = limits_node["friction_mu"].as<double>();
    if (limits_node["u_min"] && limits_node["u_min"].IsSequence()) {
      const auto raw = limits_node["u_min"].as<std::vector<double>>();
      if (!raw.empty()) {
        if (static_cast<int>(raw.size()) != nu)
          return false;
        out.u_min =
            Eigen::Map<const Eigen::VectorXd>(raw.data(), static_cast<Eigen::Index>(raw.size()));
      }
    }
    if (limits_node["u_max"] && limits_node["u_max"].IsSequence()) {
      const auto raw = limits_node["u_max"].as<std::vector<double>>();
      if (!raw.empty()) {
        if (static_cast<int>(raw.size()) != nu)
          return false;
        out.u_max =
            Eigen::Map<const Eigen::VectorXd>(raw.data(), static_cast<Eigen::Index>(raw.size()));
      }
    }
  } catch (...) {
    return false;
  }
  return true;
}

}  // namespace

bool MPCFactory::LoadSolverConfig(const YAML::Node& solver_node, MPCSolverConfig& out) noexcept {
  if (!solver_node.IsDefined())
    return true;  // leave defaults
  if (!solver_node.IsMap())
    return false;
  MPCSolverConfig cfg = out;
  try {
    if (solver_node["prim_tol"])
      cfg.prim_tol = solver_node["prim_tol"].as<double>();
    if (solver_node["dual_tol"])
      cfg.dual_tol = solver_node["dual_tol"].as<double>();
    if (solver_node["mu_init"])
      cfg.mu_init = solver_node["mu_init"].as<double>();
    if (solver_node["max_iters"])
      cfg.max_iters = solver_node["max_iters"].as<int>();
    if (solver_node["max_al_iters"])
      cfg.max_al_iters = solver_node["max_al_iters"].as<int>();
    if (solver_node["max_initial_iters"])
      cfg.max_initial_iters = solver_node["max_initial_iters"].as<int>();
    if (solver_node["max_online_al_iters"])
      cfg.max_online_al_iters = solver_node["max_online_al_iters"].as<int>();
    if (solver_node["rollout_type"] &&
        !ParseRolloutType(solver_node["rollout_type"].as<std::string>(), cfg.rollout_type))
      return false;
    if (solver_node["linear_solver"] &&
        !ParseLinearSolver(solver_node["linear_solver"].as<std::string>(), cfg.linear_solver))
      return false;
    if (solver_node["num_threads"])
      cfg.num_threads = solver_node["num_threads"].as<int>();
    if (solver_node["force_initial_condition"])
      cfg.force_initial_condition = solver_node["force_initial_condition"].as<bool>();
    if (solver_node["verbose"])
      cfg.verbose = solver_node["verbose"].as<bool>();
  } catch (...) {
    return false;
  }
  out = cfg;
  return true;
}

MPCFactoryStatus MPCFactory::Create(const YAML::Node& cfg, const RobotModelHandler& model,
                                    const PhaseContext& initial_ctx,
                                    std::unique_ptr<MPCHandlerBase>& handler_out) noexcept {
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
  if (!LoadSolverConfig(mpc["solver"], solver_cfg)) {
    status.error = MPCFactoryError::kInvalidSolverConfig;
    return status;
  }

  OCPLimits limits{};
  // The kinodynamic OCP takes its friction coefficient from its own
  // constraints section; a second one here would be silently ignored.
  const YAML::Node limits_node = static_cast<const YAML::Node&>(mpc)["limits"];
  if (!ParseLimits(limits_node, model.nu(), limits) ||
      (yaml_ocp_type == KinodynamicOCP::kType && limits_node.IsDefined() && limits_node.IsMap() &&
       static_cast<const YAML::Node&>(limits_node)["friction_mu"].IsDefined())) {
    status.error = MPCFactoryError::kInvalidLimits;
    return status;
  }

  std::unique_ptr<MPCHandlerBase> handler;
  if (yaml_ocp_type != "contact_light" && yaml_ocp_type != "contact_rich" &&
      yaml_ocp_type != KinodynamicOCP::kType) {
    status.error = MPCFactoryError::kUnknownOcpType;
    return status;
  }
  // The weights are laid onto a floating model's tangent; a fixed base has
  // none to lay them onto, and the handler's Init refuses it
  // (kFixedBaseUnsupported) before the OCP reads its config.
  KinodynamicOCPConfig ocp_cfg;
  if (yaml_ocp_type == KinodynamicOCP::kType && model.Initialised() && model.floating_base()) {
    const KinodynamicOCPConfigError ce =
        LoadKinodynamicOCPConfig(mpc["kinodynamic"], model, ocp_cfg);
    if (ce != KinodynamicOCPConfigError::kNoError) {
      status.error = MPCFactoryError::kInvalidOcpConfig;
      status.ocp_config_error = ce;
      return status;
    }
  }
  try {
    if (yaml_ocp_type == "contact_light") {
      handler = std::make_unique<ContactLightMPC>();
    } else if (yaml_ocp_type == "contact_rich") {
      handler = std::make_unique<ContactRichMPC>();
    } else {
      handler = std::make_unique<KinodynamicMPC>(std::move(ocp_cfg));
    }
  } catch (...) {
    status.error = MPCFactoryError::kHandlerInitFailed;  // an allocation failed
    return status;
  }

  const MPCInitError ie = handler->Init(solver_cfg, model, limits, initial_ctx);
  if (ie != MPCInitError::kNoError) {
    status.error = MPCFactoryError::kHandlerInitFailed;
    status.init_error = ie;
    return status;
  }

  handler_out = std::move(handler);
  return status;  // kNoError
}

}  // namespace rtc::mpc
