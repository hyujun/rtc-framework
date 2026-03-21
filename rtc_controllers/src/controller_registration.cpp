// ── Built-in controller registration ──────────────────────────────────────────
//
// Registers the 4 generic (robot-agnostic) controllers into the global
// ControllerRegistry at static-init time.  The ForceBuiltinControllerRegistration()
// function must be called from main() to prevent the linker from stripping
// this translation unit when linking static libraries.

#include "rtc_controller_interface/controller_registry.hpp"

#include "rtc_controllers/indirect/p_controller.hpp"
#include "rtc_controllers/indirect/clik_controller.hpp"
#include "rtc_controllers/direct/joint_pd_controller.hpp"
#include "rtc_controllers/direct/operational_space_controller.hpp"

RTC_REGISTER_CONTROLLER(
  p_controller, "indirect/", "rtc_controllers",
  std::make_unique<rtc::PController>(urdf))

RTC_REGISTER_CONTROLLER(
  joint_pd_controller, "direct/", "rtc_controllers",
  std::make_unique<rtc::JointPDController>(urdf))

RTC_REGISTER_CONTROLLER(
  clik_controller, "indirect/", "rtc_controllers",
  std::make_unique<rtc::ClikController>(urdf, rtc::ClikController::Gains{}))

RTC_REGISTER_CONTROLLER(
  operational_space_controller, "direct/", "rtc_controllers",
  std::make_unique<rtc::OperationalSpaceController>(
    urdf, rtc::OperationalSpaceController::Gains{}))

namespace rtc
{
void ForceBuiltinControllerRegistration() {}
}  // namespace rtc
