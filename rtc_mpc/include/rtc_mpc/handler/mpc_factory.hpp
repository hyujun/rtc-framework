#ifndef RTC_MPC_HANDLER_MPC_FACTORY_HPP_
#define RTC_MPC_HANDLER_MPC_FACTORY_HPP_

/// @file mpc_factory.hpp
/// @brief YAML-driven factory that dispatches on `ocp_type` to build a
///        concrete `MPCHandlerBase` subclass.
///
/// Expected YAML schema (loaded once at construction-time):
///
/// ```yaml
/// mpc:
///   ocp_type: "light_contact"   # or "contact_rich"
///   solver:
///     prim_tol: 1.0e-3
///     dual_tol: 1.0e-2
///     mu_init: 1.0e-2
///     max_iters: 30
///     max_al_iters: 20
///     verbose: false
///   limits:
///     friction_mu: 0.7          # only read by contact_rich
///     n_friction_facets: 4      # reserved (see OCPLimits docs)
///     u_min: []                 # optional; size nu or empty
///     u_max: []                 # optional; size nu or empty
/// ```
///
/// Unknown `ocp_type` strings are rejected — never silently fall back.
/// Robot identity is not part of this YAML; the caller supplies the
/// already-constructed `RobotModelHandler` + `PhaseContext`.

#include "rtc_mpc/handler/mpc_handler_base.hpp"

#include <yaml-cpp/yaml.h>

#include <memory>

namespace rtc::mpc {

/// @brief Failure modes for @ref MPCFactory::Create.
enum class MPCFactoryError {
  kNoError = 0,
  kMissingOcpTypeKey,   ///< YAML has no `mpc.ocp_type` entry
  kUnknownOcpType,      ///< `ocp_type` string not in the dispatch table
  kInvalidSolverConfig, ///< nested `solver:` subtree failed validation
  kInvalidLimits,       ///< u_min / u_max size mismatch vs. model
  kHandlerInitFailed,   ///< concrete handler's `Init` rejected (embedded
                        ///< MPCInitError available via the `last_*` probes)
};

/// @brief Holder for the out-of-band error context the factory cannot fit
///        in the primary enum. Written by `Create` on failure paths.
struct MPCFactoryStatus {
  MPCFactoryError error{MPCFactoryError::kNoError};
  MPCInitError init_error{
      MPCInitError::kNoError}; ///< set on kHandlerInitFailed
};

/// @brief Static factory — no state, no instance.
class MPCFactory {
public:
  MPCFactory() = delete;
  ~MPCFactory() = delete;

  /// @brief Parse YAML, construct the matching `MPCHandlerBase` subclass,
  ///        and run its `Init` against the supplied model + initial
  ///        phase context. On failure, `handler_out` stays null and the
  ///        returned status reflects the failure class.
  ///
  /// @param cfg           YAML root node (the `mpc:` key and everything
  ///                      below); accepts either a root with a nested
  ///                      `mpc:` map or the nested map directly.
  /// @param model         already-initialised `RobotModelHandler`
  /// @param initial_ctx   first phase context; its `ocp_type` MUST match
  ///                      the YAML's `ocp_type` (guard against config
  ///                      drift between phase manager and solver).
  /// @param handler_out   out-param: the created handler on success.
  /// @return              structured status; `kNoError` on success.
  [[nodiscard]] static MPCFactoryStatus
  Create(const YAML::Node &cfg, const RobotModelHandler &model,
         const PhaseContext &initial_ctx,
         std::unique_ptr<MPCHandlerBase> &handler_out) noexcept;
};

} // namespace rtc::mpc

#endif // RTC_MPC_HANDLER_MPC_FACTORY_HPP_
