// ── UR5e demo controller registration ─────────────────────────────────────────
//
// Registers DemoJointController and DemoTaskController into the global
// ControllerRegistry.  ForceDemoControllerRegistration() must be called
// from main() to prevent the linker from stripping this translation unit.

#include "rtc_controller_interface/controller_registry.hpp"

#include "ur5e_bringup/controllers/demo_joint_controller.hpp"
#include "ur5e_bringup/controllers/demo_task_controller.hpp"

RTC_REGISTER_CONTROLLER(
  demo_joint_controller, "", "ur5e_bringup",
  std::make_unique<ur5e_bringup::DemoJointController>(urdf))

RTC_REGISTER_CONTROLLER(
  demo_task_controller, "", "ur5e_bringup",
  std::make_unique<ur5e_bringup::DemoTaskController>(
    urdf, ur5e_bringup::DemoTaskController::Gains{}))

namespace ur5e_bringup
{
void ForceDemoControllerRegistration() {}
}  // namespace ur5e_bringup
