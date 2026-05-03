// ── UR5e demo controller registration ─────────────────────────────────────────
//
// Registers DemoJointController and DemoTaskController into the global
// ControllerRegistry.  The library is linked with --whole-archive so the
// linker preserves this translation unit without a Force function.

#include "rtc_controller_interface/controller_registry.hpp"
#include "integrated_bringup/controllers/demo_joint_controller.hpp"
#include "integrated_bringup/controllers/demo_task_controller.hpp"
#include "integrated_bringup/controllers/demo_wbc_controller.hpp"

RTC_REGISTER_CONTROLLER(demo_joint_controller, "", "integrated_bringup",
                        std::make_unique<integrated_bringup::DemoJointController>(urdf))

RTC_REGISTER_CONTROLLER(demo_task_controller, "", "integrated_bringup",
                        std::make_unique<integrated_bringup::DemoTaskController>(
                            urdf, integrated_bringup::DemoTaskController::Gains{}))

RTC_REGISTER_CONTROLLER(demo_wbc_controller, "", "integrated_bringup",
                        std::make_unique<integrated_bringup::DemoWbcController>(urdf))
