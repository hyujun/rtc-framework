#ifndef RTC_MPC_HANDLER_LIGHT_CONTACT_MPC_HPP_
#define RTC_MPC_HANDLER_LIGHT_CONTACT_MPC_HPP_

/// @file contact_light_mpc.hpp
/// @brief Concrete `MPCHandlerBase` wrapping a `ContactLightOCP` +
///        Aligator `SolverProxDDP`. One instance per contact-light phase
///        sequence; cross-mode transitions are the factory's job.

#include "rtc_mpc/handler/mpc_handler_base.hpp"
#include "rtc_mpc/ocp/contact_light_ocp.hpp"

#include <memory>

namespace rtc::mpc {

namespace internal {
class MPCHandlerCore;
}  // namespace internal

class ContactLightMPC : public MPCHandlerBase {
 public:
  ContactLightMPC();
  ~ContactLightMPC() override;

  ContactLightMPC(const ContactLightMPC&) = delete;
  ContactLightMPC& operator=(const ContactLightMPC&) = delete;
  ContactLightMPC(ContactLightMPC&&) = delete;
  ContactLightMPC& operator=(ContactLightMPC&&) = delete;

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

 private:
  ContactLightOCP ocp_;
  std::unique_ptr<internal::MPCHandlerCore> core_;
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_HANDLER_LIGHT_CONTACT_MPC_HPP_
