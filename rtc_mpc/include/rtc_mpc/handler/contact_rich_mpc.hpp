#ifndef RTC_MPC_HANDLER_CONTACT_RICH_MPC_HPP_
#define RTC_MPC_HANDLER_CONTACT_RICH_MPC_HPP_

/// @file contact_rich_mpc.hpp
/// @brief Concrete `MPCHandlerBase` wrapping a `ContactRichOCP`. Exposes
///        the same grasp-quality provider seam as the underlying OCP so
///        `ur5e_bringup` (Phase 7) can inject a concrete provider without
///        reaching through to the OCP.

#include "rtc_mpc/handler/mpc_handler_base.hpp"
#include "rtc_mpc/ocp/contact_rich_ocp.hpp"

#include <memory>

namespace rtc::mpc {

namespace internal {
class MPCHandlerCore;
} // namespace internal

class GraspQualityResidualProvider;

class ContactRichMPC : public MPCHandlerBase {
public:
  ContactRichMPC();
  ~ContactRichMPC() override;

  ContactRichMPC(const ContactRichMPC &) = delete;
  ContactRichMPC &operator=(const ContactRichMPC &) = delete;
  ContactRichMPC(ContactRichMPC &&) = delete;
  ContactRichMPC &operator=(ContactRichMPC &&) = delete;

  [[nodiscard]] MPCInitError
  Init(const MPCSolverConfig &solver_cfg, const RobotModelHandler &model,
       const OCPLimits &limits,
       const PhaseContext &initial_ctx) noexcept override;

  [[nodiscard]] MPCSolveError Solve(const PhaseContext &ctx,
                                    const MPCStateSnapshot &state,
                                    MPCSolution &out) noexcept override;

  [[nodiscard]] bool Initialised() const noexcept override;

  [[nodiscard]] std::string_view ocp_type() const noexcept override {
    return ocp_.ocp_type();
  }

  [[nodiscard]] int horizon_length() const noexcept override;
  [[nodiscard]] int nq() const noexcept override;
  [[nodiscard]] int nv() const noexcept override;
  [[nodiscard]] int nu() const noexcept override;
  [[nodiscard]] int n_contact_vars() const noexcept override;

  void SeedWarmStart(const MPCSolution &prev_solution) noexcept override;

  /// @brief Forward to `ContactRichOCP::SetGraspQualityProvider`. Takes
  ///        effect on the next `Init` / rebuild; does not invalidate the
  ///        current problem.
  void
  SetGraspQualityProvider(GraspQualityResidualProvider *provider) noexcept {
    ocp_.SetGraspQualityProvider(provider);
  }

private:
  ContactRichOCP ocp_;
  std::unique_ptr<internal::MPCHandlerCore> core_;
};

} // namespace rtc::mpc

#endif // RTC_MPC_HANDLER_CONTACT_RICH_MPC_HPP_
