#ifndef RTC_MPC_HANDLER_KINODYNAMIC_MPC_HPP_
#define RTC_MPC_HANDLER_KINODYNAMIC_MPC_HPP_

/// @file kinodynamic_mpc.hpp
/// @brief Concrete `MPCHandlerBase` over a `KinodynamicOCP` (floating base,
///        `ocp_type: kinodynamic`) and Aligator's `SolverProxDDP`: the
///        shared handler core does the solve; this class only owns the OCP.
///
/// Every `Solve` needs a `PhaseContext::horizon` — without one the OCP
/// refuses the tick (`kOCPRebuildFailed`). A fixed-base model is refused at
/// `Init` (`kFixedBaseUnsupported`).

#include "rtc_mpc/handler/mpc_handler_base.hpp"
#include "rtc_mpc/ocp/kinodynamic_ocp.hpp"

#include <memory>

namespace rtc::mpc {

namespace internal {
class MPCHandlerCore;
}  // namespace internal

class KinodynamicMPC : public MPCHandlerBase {
 public:
  /// @param cfg  `mpc.kinodynamic:` loaded for the model `Init` will get
  explicit KinodynamicMPC(KinodynamicOCPConfig cfg);
  ~KinodynamicMPC() override;

  KinodynamicMPC(const KinodynamicMPC&) = delete;
  KinodynamicMPC& operator=(const KinodynamicMPC&) = delete;
  KinodynamicMPC(KinodynamicMPC&&) = delete;
  KinodynamicMPC& operator=(KinodynamicMPC&&) = delete;

  [[nodiscard]] MPCInitError Init(const MPCSolverConfig& solver_cfg, const RobotModelHandler& model,
                                  const OCPLimits& limits,
                                  const PhaseContext& initial_ctx) noexcept override;

  [[nodiscard]] MPCSolveError Solve(const PhaseContext& ctx, const MPCStateSnapshot& state,
                                    MPCSolution& out) noexcept override;

  [[nodiscard]] bool Initialised() const noexcept override;

  [[nodiscard]] std::string_view ocp_type() const noexcept override { return ocp_.ocp_type(); }

  [[nodiscard]] int horizon_length() const noexcept override;
  [[nodiscard]] int nq() const noexcept override;
  [[nodiscard]] int nv() const noexcept override;
  [[nodiscard]] int nu() const noexcept override;
  [[nodiscard]] int n_contact_vars() const noexcept override;

  void SeedWarmStart(const MPCSolution& prev_solution) noexcept override;

  /// @return the OCP, for inspection (its problem, its config).
  [[nodiscard]] KinodynamicOCP& ocp() noexcept { return ocp_; }

 private:
  KinodynamicOCP ocp_;
  std::unique_ptr<internal::MPCHandlerCore> core_;
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_HANDLER_KINODYNAMIC_MPC_HPP_
